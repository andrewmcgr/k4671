use core::cmp::min;
use core::sync::atomic::AtomicBool;

use crate::{KLIPPER_TRANSPORT, LED_STATE};
use crate::{LedState, USB_DOORBELL};
use anchor::{FifoBuffer, InputBuffer, SliceInputBuffer};
use defmt::*;
use embassy_futures::join::join;
use embassy_futures::select::{Either5, Either6, select, select5, select6};
use embassy_stm32::i2c::RxDma;
use embassy_stm32::uid;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, ControlChanged, Receiver, Sender, State};
use embassy_usb::driver::Driver;
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use embedded_io_async::Write;
use heapless::Vec;
use heapless::spsc::Consumer;

pub const ANCHOR_PIPE_SIZE: usize = 2048;
pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type AnchorPipe = Pipe<CS, ANCHOR_PIPE_SIZE>;
pub type AnchorMutex<T> = Mutex<CS, T>;

pub static ANCHOR_RX_CONNECTED: AtomicBool = AtomicBool::new(false);
pub static ANCHOR_TX_CONNECTED: AtomicBool = AtomicBool::new(false);

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

    /// Run the USB anchor using the state and USB driver.
    pub async fn run<'d, D>(
        &'d mut self,
        state: &'d mut AnchorState<'d>,
        in_pipe: &'d AnchorPipe,
        out_pipe: &'d mut Consumer<'d, Vec<u8, 64>>,
        driver: D,
    ) where
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
        let run_fut = device.run();
        let class_fut =
            self.run_anchor_class(in_pipe, out_pipe, &mut sender, &mut receiver, &mut control);
        join(run_fut, class_fut).await;
    }

    async fn run_anchor_class<'d, D>(
        &mut self,
        _in_pipe: &'d AnchorPipe,
        out_pipe: &'d mut Consumer<'d, Vec<u8, 64>>,
        sender: &mut Sender<'d, D>,
        receiver: &mut Receiver<'d, D>,
        control: &mut ControlChanged<'d>,
    ) where
        D: Driver<'d>,
    {
        // let mut out_fut = async || -> Result<(), Disconnected> {
        //     ANCHOR_TX_CONNECTED.store(false, core::sync::atomic::Ordering::Relaxed);
        //     sender.wait_connection().await;
        //     ANCHOR_TX_CONNECTED.store(true, core::sync::atomic::Ordering::Relaxed);
        //     loop {
        //         LED_STATE.signal(LedState::N(4));
        //         let _ = USB_DOORBELL.wait().await;
        //         while let Some(v) = out_pipe.dequeue() {
        //             sender.write_packet(&v).await?;
        //         }
        //     }
        // };
        let mut reciever_fut = async || -> Result<(), Disconnected> {
            let mut reciever_buf: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
            let mut state = crate::State::new();
            let tmc_sender = &crate::TMC_CMD;
            // let mut trsync_receiver = crate::TRSYNC_WATCH.receiver().unwrap();
            let mut trsync_ticks = Instant::MAX;

            type RxBuf = FifoBuffer<{ MAX_PACKET_SIZE as usize * 2 }>;
            let mut rx_buf: RxBuf = RxBuf::new();
            ANCHOR_RX_CONNECTED.store(false, core::sync::atomic::Ordering::Relaxed);
            receiver.wait_connection().await;
            ANCHOR_RX_CONNECTED.store(true, core::sync::atomic::Ordering::Relaxed);

            let move_period = Duration::from_hz(2500);

            let mut move_ticks = Instant::now() + move_period;

            loop {
                let res = select5(
                    receiver.read_packet(&mut reciever_buf),
                    control.control_changed(),
                    Timer::after(move_period),
                    Timer::at(trsync_ticks),
                    USB_DOORBELL.wait(),
                )
                .await;
                match &res {
                    Either5::Third(_) => {}
                    _ => info!("Anchor event {}", defmt::Debug2Format(&res)),
                }

                match res {
                    // USB disconnect
                    Either5::First(Err(e)) => return Err(e.into()),
                    // Received data or need to send
                    Either5::First(_) => {
                        LED_STATE.signal(LedState::N(1));
                        if let Either5::First(Ok(len)) = res {
                            debug!("Anchor In {:x}", &reciever_buf[..len]);
                            info!("Anchor In {} bytes", len);
                            rx_buf.extend(&reciever_buf[..len]);
                            if !rx_buf.is_empty() {
                                let mut wrap = SliceInputBuffer::new(rx_buf.data());
                                KLIPPER_TRANSPORT.receive(&mut wrap, &mut state);
                                let consumed = rx_buf.len() - wrap.available();
                                rx_buf.pop(consumed);
                            }
                        }
                        // Pump USB
                        while let Some(v) = out_pipe.dequeue() {
                            sender.write(&v).await?;
                            if v.len() == MAX_PACKET_SIZE as usize {
                                // USB full packet, send another to flush
                                sender.write_packet(&[]).await?;
                            }
                        }
                        // Have trsync check if it needs to do something
                        trsync_ticks = Instant::MIN;
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
                        move_ticks = Instant::MAX;
                        for stepper in state.steppers.iter_mut() {
                            if let (next_time, Some(cmd)) =
                                crate::process_moves(stepper, Instant::now() + move_period)
                            {
                                // debug!("Sending TMC command {:?}", cmd);
                                info!("TMC Cmd {:?}", defmt::Debug2Format(&cmd));
                                tmc_sender.enqueue(cmd).ok();
                                info!("TMC Cmd enqueued");
                                let t = next_time.unwrap_or_else(|| Instant::now() + move_period);
                                move_ticks = min(move_ticks, t);
                            }
                        }
                    }
                    // TrSync state changed or timer expired or must pump USB
                    _ => {
                        LED_STATE.signal(LedState::N(2));
                        info!("Processing TrSync");
                        trsync_ticks = Instant::MAX;
                        for t in state.trsync.iter_mut() {
                            if t.can_trigger
                                && let Some(oid) = t.oid
                            {
                                trsync_ticks = min(trsync_ticks, t.process_trsync(oid));
                            }
                        }
                        // Pump USB
                        while let Some(v) = out_pipe.dequeue() {
                            sender.write_packet(&v).await?;
                            if v.len() == MAX_PACKET_SIZE as usize {
                                // USB full packet, send another to flush
                                sender.write_packet(&[]).await?;
                            }
                        }
                    }
                };
            }
        };

        loop {
            // LED_STATE.signal(Connecting);
            // let _ = select(out_fut(), reciever_fut()).await;
            let _ = reciever_fut().await;

            // LED_STATE.signal(Error);
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
