use embassy_futures::join::join;
use embassy_sync::mutex::Mutex;
use embassy_stm32::uid;
use embassy_sync::pipe::Pipe;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, Config};

pub const ANCHOR_PIPE_SIZE: usize = 1024;
pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type AnchorPipe = Pipe<CS, ANCHOR_PIPE_SIZE>;
pub type AnchorMutex<T> = Mutex<CS, T>;

/// The anchor state containing buffers that must live as long as the USB peripheral.
pub struct AnchorState<'d> {
    state: State<'d>,
    config_descriptor: [u8; 128],
    bos_descriptor: [u8; 16],
    msos_descriptor: [u8; 256],
    control_buf: [u8; 64],
}

impl<'d> AnchorState<'d> {
    /// Create a new instance of the anchor state.
    pub fn new() -> Self {
        Self {
            state: State::new(),
            config_descriptor: [0; 128],
            bos_descriptor: [0; 16],
            msos_descriptor: [0; 256],
            control_buf: [0; 64],
        }
    }
}

/// The packet size used in the usb anchor, to be used with `create_future_from_class`
pub const MAX_PACKET_SIZE: u8 = 64;

/// The anchor handle, which contains a pipe with configurable size for buffering log messages.
pub struct UsbAnchor {}

impl UsbAnchor {
    /// Create a new anchor instance.
    pub fn new() -> Self {
        Self {}
    }

    /// Run the USB anchor using the state and USB driver. Never returns.
    pub async fn run<'d, D>(
        &'d mut self,
        state: &'d mut AnchorState<'d>,
        in_pipe: &'d AnchorPipe,
        out_pipe: &'d AnchorPipe,
        driver: D,
    ) -> !
    where
        D: Driver<'d>,
        Self: 'd,
    {
        let mut config = Config::new(0x1209, 0x4671);
        config.manufacturer = Some("k4671");
        config.product = Some("K4671 Motor Driver");
        config.serial_number = Some(uid::uid_hex());
        config.max_power = 100;
        config.max_packet_size_0 = MAX_PACKET_SIZE;

        let mut builder = Builder::new(
            driver,
            config,
            &mut state.config_descriptor,
            &mut state.bos_descriptor,
            &mut state.msos_descriptor,
            &mut state.control_buf,
        );

        // Create classes on the builder.
        let class = CdcAcmClass::new(&mut builder, &mut state.state, MAX_PACKET_SIZE as u16);
        let (mut sender, mut receiver) = class.split();

        // Build the builder.
        let mut device = builder.build();
        loop {
            let run_fut = device.run();
            let class_fut = self.run_anchor_class(in_pipe, out_pipe, &mut sender, &mut receiver);
            join(run_fut, class_fut).await;
        }
    }

    async fn run_anchor_class<'d, D>(
        &mut self,
        in_pipe: &'d AnchorPipe,
        out_pipe: &'d AnchorPipe,
        sender: &mut Sender<'d, D>,
        receiver: &mut Receiver<'d, D>,
    ) where
        D: Driver<'d>,
    {
        let out_fut = async {
            let mut rx: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            sender.wait_connection().await;
            loop {
                let len = out_pipe.read(&mut rx[..]).await;
                let _ = sender.write_packet(&rx[..len]).await;
                if len as u8 == MAX_PACKET_SIZE {
                    let _ = sender.write_packet(&[]).await;
                }
            }
        };
        let reciever_fut = async {
            let mut reciever_buf: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            receiver.wait_connection().await;
            loop {
                let _ = receiver.read_packet(&mut reciever_buf).await.unwrap();
                in_pipe.write_all(&mut reciever_buf[..]).await;
            }
        };

        join(out_fut, reciever_fut).await;
    }
}

/// A writer that writes to the USB buffer.
pub struct PipeWriter<'d, const N: usize>(&'d Pipe<CS, N>);

impl<'d, const N: usize> core::fmt::Write for PipeWriter<'d, N> {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        // The Pipe is implemented in such way that we cannot
        // write across the wraparound discontinuity.
        let b = s.as_bytes();
        if let Ok(n) = self.0.try_write(b) {
            if n < b.len() {
                // We wrote some data but not all, attempt again
                // as the reason might be a wraparound in the
                // ring buffer, which resolves on second attempt.
                let _ = self.0.try_write(&b[n..]);
            }
        }
        Ok(())
    }
}
