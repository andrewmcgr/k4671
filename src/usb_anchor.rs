#![no_std]

use core::fmt::Write as _;
use core::future::Future;

use embassy_futures::join::join;
use embassy_sync::pipe::Pipe;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, Config};
use embassy_stm32::uid;

type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// A trait that can be implemented and then passed to the
pub trait ReceiverHandler {
    /// Data comes in from the serial port with each command and runs this function
    fn handle_data(&self, data: &[u8]) -> impl Future<Output = ()> + Send;

    /// Create a new instance of the Handler
    fn new() -> Self;
}

/// Use this Handler if you don't wish to use any handler
pub struct DummyHandler;

impl ReceiverHandler for DummyHandler {
    async fn handle_data(&self, _data: &[u8]) {}
    fn new() -> Self {
        Self {}
    }
}

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
pub struct UsbAnchor<const N: usize, T: ReceiverHandler + Send + Sync> {
    buffer: Pipe<CS, N>,
    recieve_handler: Option<T>,
}

impl<const N: usize, T: ReceiverHandler + Send + Sync> UsbAnchor<N, T> {
    /// Create a new anchor instance.
    pub const fn new() -> Self {
        Self {
            buffer: Pipe::new(),
            recieve_handler: None,
        }
    }

    /// Add a command handler to the anchor
    pub fn with_handler(&mut self, handler: T) {
        self.recieve_handler = Some(handler);
    }

    /// Run the USB anchor using the state and USB driver. Never returns.
    pub async fn run<'d, D>(&'d self, state: &'d mut AnchorState<'d>, driver: D) -> !
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
            let class_fut = self.run_anchor_class(&mut sender, &mut receiver);
            join(run_fut, class_fut).await;
        }
    }

    async fn run_anchor_class<'d, D>(&self, sender: &mut Sender<'d, D>, receiver: &mut Receiver<'d, D>)
    where
        D: Driver<'d>,
    {
        let out_fut = async {
            let mut rx: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            sender.wait_connection().await;
            loop {
                let len = self.buffer.read(&mut rx[..]).await;
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
                let n = receiver.read_packet(&mut reciever_buf).await.unwrap();
                match &self.recieve_handler {
                    Some(handler) => {
                        let data = &reciever_buf[..n];
                        handler.handle_data(data).await;
                    }
                    None => (),
                }
            }
        };

        join(out_fut, reciever_fut).await;
    }

}

/// A writer that writes to the USB anchor output buffer.
pub struct Writer<'d, const N: usize>(&'d Pipe<CS, N>);

impl<'d, const N: usize> core::fmt::Write for Writer<'d, N> {
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
