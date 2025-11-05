use crate::LedState::{Connecting, Error};
use crate::{KLIPPER_TRANSPORT, LED_STATE};
use anchor::{FifoBuffer, InputBuffer, SliceInputBuffer};
use defmt::*;
use embassy_futures::join::join;
use embassy_futures::select::{Either5, select, select5};
use embassy_stm32::uid;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender, State};
use embassy_usb::driver::Driver;
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};

pub const ANCHOR_PIPE_SIZE: usize = 2048;
pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type AnchorPipe = Pipe<CS, ANCHOR_PIPE_SIZE>;
pub type AnchorMutex<T> = Mutex<CS, T>;

// pub static ANCHOR_MUTEX: AnchorMutex<()> = AnchorMutex::new(());

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => self::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
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
        info!("Hello Anchor USB!");
        let mut config = Config::new(0x1209, 0x4671);
        config.manufacturer = Some("k4671");
        config.product = Some("K4671 Motor Driver");
        config.serial_number = Some(uid::uid_hex());
        config.max_power = 500;
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
        let (mut sender, mut receiver, mut control) = class.split_with_control();

        // Build the builder.
        let mut device = builder.build();
        loop {
            let run_fut = device.run();
            let class_fut =
                self.run_anchor_class(in_pipe, out_pipe, &mut sender, &mut receiver, &mut control);
            join(run_fut, class_fut).await;
        }
    }

    async fn run_anchor_class<'d, D>(
        &mut self,
        _in_pipe: &'d AnchorPipe,
        out_pipe: &'d AnchorPipe,
        sender: &mut Sender<'d, D>,
        receiver: &mut Receiver<'d, D>,
        control: &mut ControlChanged<'d>,
    ) where
        D: Driver<'d>,
    {
        let mut out_fut = async || -> Result<(), Disconnected> {
            let mut rx: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            sender.wait_connection().await;
            loop {
                let len = out_pipe.read(&mut rx[..]).await;
                debug!("Anchor Out {:x}", &rx[..len]);
                let _ = sender.write_packet(&rx[..len]).await?;
                if len as u8 == MAX_PACKET_SIZE {
                    let _ = sender.write_packet(&[]).await?;
                }
                Timer::after_micros(10).await;
                // yield_now().await;
            }
        };
        let mut reciever_fut = async || -> Result<(), Disconnected> {
            let mut reciever_buf: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            let mut state = crate::State::new();
            let tmc_sender = &crate::TMC_CMD;
            let mut trsync_receiver = crate::TRSYNC_WATCH.receiver().unwrap();
            let mut trsync_ticks = Instant::MAX;

            type RxBuf = FifoBuffer<{ MAX_PACKET_SIZE as usize * 2 }>;
            let mut rx_buf: RxBuf = RxBuf::new();
            receiver.wait_connection().await;

            let mut move_ticker = Ticker::every(Duration::from_micros(500));

            loop {
                let res = select5(
                    receiver.read_packet(&mut reciever_buf),
                    control.control_changed(),
                    move_ticker.next(),
                    trsync_receiver.changed(),
                    Timer::at(trsync_ticks),
                )
                .await;
                match res {
                    // USB disconnect
                    Either5::First(Err(e)) => return Err(e.into()),
                    // Received data
                    Either5::First(Ok(len)) => {
                        debug!("Anchor In {:x}", &reciever_buf[..len]);
                        rx_buf.extend(&reciever_buf[..len]);
                        if !rx_buf.is_empty() {
                            let mut wrap = SliceInputBuffer::new(rx_buf.data());
                            KLIPPER_TRANSPORT.receive(&mut wrap, &mut state);
                            let consumed = rx_buf.len() - wrap.available();
                            rx_buf.pop(consumed);
                        }
                    }
                    // DFU request
                    Either5::Second(_) => {
                        if receiver.line_coding().data_rate() == 1200 {
                            // Special case: 1200 baud on a CDC ACM port is the "signal to
                            // reboot to bootloader" in the Arduino world.
                            dfu::enter_dfu_mode();
                            // Unreachable, as enter_dfu_mode does not return.
                        }
                    }
                    // Move ticker
                    Either5::Third(_) => {
                        for stepper in state.steppers.iter_mut() {
                            if let Some(cmd) = crate::process_moves(
                                stepper,
                                Instant::now() + Duration::from_micros(1100),
                            ) {
                                tmc_sender.enqueue(cmd).ok();
                            }
                        }
                    }
                    // TrSync state changed or timer expired
                    Either5::Fourth(_) | Either5::Fifth(_) => {
                        trsync_ticks = Instant::MAX;
                        for t in state.trsync.iter_mut() {
                            if let Some(oid) = t.oid {
                                let time = t.process_trsync(oid);
                                if time < trsync_ticks {
                                    trsync_ticks = time;
                                }
                            }
                        }
                    }
                };
            }
        };

        loop {
            LED_STATE.signal(Connecting);
            let _ = select(out_fut(), reciever_fut()).await;
            LED_STATE.signal(Error);
            Timer::after_millis(900).await;
        }
    }
}

/// A writer that writes to the USB buffer.
#[allow(dead_code)]
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
