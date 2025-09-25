use crate::TMC_CMD;
use defmt::*;
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::{Mutex as BlockingMutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant};
use heapless::Deque;
use tmc4671::*;

pub struct MutexWrapper;

impl crate::target_queue::Mutex for MutexWrapper {
    type Inner<T> = BlockingMutex<CriticalSectionRawMutex, T>;

    fn new<T>(val: T) -> Self::Inner<T> {
        BlockingMutex::new(val)
    }

    fn lock<T, R>(inner: &Self::Inner<T>, f: impl FnOnce(&T) -> R) -> R {
        inner.lock(f)
    }
}

pub type TargetQueue = crate::target_queue::TargetQueue<MutexWrapper, 2000>;

#[derive(Debug, defmt::Format, Copy, Clone, Eq, PartialEq, PartialOrd, Ord)]
pub enum Direction {
    Forward,
    Backward,
}

#[derive(Debug, Copy, Clone, defmt::Format)]
#[repr(C)]
pub struct Move {
    interval: u32,
    count: u16,
    add: i16,
    direction: Direction,
}

impl Move {
    fn total_time(&self) -> Duration {
        self.time_after_steps(self.count)
    }

    fn time_after_steps(&self, steps: u16) -> Duration {
        if steps == 0 {
            return Duration::from_micros(0);
        }
        let base = (steps as u64) * (self.interval as u64);
        let accel = (self.add as i32) * (steps as i32 - 1) * (steps as i32) / 2;
        Duration::from_micros(base.wrapping_add(accel as u64))
    }

    fn steps_before_time(&self, target: Duration) -> u16 {
        let mut l = 0;
        let mut r = self.count;
        while l <= r {
            let m = (r - l) / 2 + l;
            let v = self.time_after_steps(m);
            if v == target {
                return m;
            } else if v >= target {
                r = m - 1;
            } else {
                l = m + 1;
            }
        }
        l - 1
    }

    fn advance(&self, steps: u16) -> Move {
        let steps = steps.clamp(0, self.count);
        Move {
            interval: self
                .interval
                .wrapping_add(((self.add as i32) * (steps as i32)) as u32),
            count: self.count - steps,
            add: self.add,
            direction: self.direction,
        }
    }
}

#[derive(Debug)]
struct State {
    last_step: Instant,
    position: u32,
}

impl State {
    /// Advances the state by the given move, up to maximum time
    fn advance(&mut self, cmd: &Move, up_to_time: Instant) -> AdvanceResult {
        let next_step =
            Instant::from_micros(self.last_step.as_micros().wrapping_add(cmd.interval.into()));
        if next_step > up_to_time {
            return AdvanceResult::FutureMove;
        }
        // If the next step is within our window, consume one step
        self.step(cmd.direction, 1);
        self.last_step = next_step;
        let cmd = cmd.advance(1);
        if cmd.count == 0 {
            return AdvanceResult::Consumed; // We consumed the entire thing.
        }

        // Now see if we can apply more steps
        let available_time = match up_to_time.checked_duration_since(self.last_step) {
            Some(t) => t,
            None => return AdvanceResult::Partial(cmd),
        };

        let total_time = cmd.total_time();
        if total_time < available_time {
            // Apply the full move
            self.last_step += total_time;
            self.step(cmd.direction, cmd.count as u32);
            return AdvanceResult::Consumed;
        }

        let steps_before = cmd.steps_before_time(available_time);
        if steps_before == 0 {
            return AdvanceResult::Partial(cmd); // Fast path: nothing can be applied
        }
        // Slow path: apply the time and number of steps before `steps_before` and return the
        // remaining move.
        self.last_step += cmd.time_after_steps(steps_before);
        self.step(cmd.direction, steps_before as u32);
        AdvanceResult::Partial(cmd.advance(steps_before))
    }

    fn step(&mut self, direction: Direction, count: u32) {
        self.position = if direction == Direction::Forward {
            self.position.wrapping_add(count)
        } else {
            self.position.wrapping_sub(count)
        }
    }
}

#[derive(Debug, defmt::Format, Copy, Clone)]
enum AdvanceResult {
    FutureMove,
    Consumed,
    Partial(Move),
}

pub trait Callbacks {
    fn append(&mut self, time: Instant, value: u32);
    fn update_last(&mut self, time: Instant, value: u32);
    fn can_append(&self) -> bool;
}

#[derive(Debug)]
pub struct EmulatedStepper<T, const N: usize> {
    queue: heapless::Deque<Move, N>,
    current_move: Option<Move>,
    next_direction: Direction,
    state: State,
    target_time: T,
    callback_state: CallbackState,
    reset_target: Option<u32>,
    pub stepper_oid: Option<u8>,
    pub stepper_enable_oid: Option<u8>,
}

#[derive(Debug)]
struct CallbackState {
    last_append: (Instant, u32),
    incomplete: bool,
}

impl CallbackState {
    fn append(&mut self, next_time: Instant, position: u32, callbacks: &mut impl Callbacks) {
        callbacks.append(next_time, position);
        self.last_append = (next_time, position);
    }

    fn update(&mut self, position: u32, callbacks: &mut impl Callbacks) {
        let pos = position;
        if pos != self.last_append.1 {
            callbacks.update_last(self.last_append.0, pos);
        }
    }

    fn emit(&mut self, next_time: Instant, position: u32, callbacks: &mut impl Callbacks) {
        if self.incomplete {
            self.update(position, callbacks);
        } else {
            self.append(next_time, position, callbacks);
        }
    }

    fn can_append(&self, callbacks: &impl Callbacks) -> bool {
        callbacks.can_append()
    }
}

impl<T: tmc4671::TimeIterator, const N: usize> EmulatedStepper<T, N> {
    pub fn new(target_time: T) -> Self {
        Self {
            queue: Deque::new(),
            stepper_oid: None,
            stepper_enable_oid: None,
            current_move: None,
            next_direction: Direction::Forward,
            state: State {
                last_step: Instant::from_micros(0),
                position: 0,
            },
            target_time,
            callback_state: CallbackState {
                last_append: (Instant::from_micros(0), 0),
                incomplete: true,
            },
            reset_target: None,
        }
    }

    pub fn reset_clock(&mut self, time: Instant) {
        self.state.last_step = time;
    }

    pub fn current_position(&self) -> i32 {
        self.state.position as i32
    }

    pub fn reset_target(&mut self, new_target: u32) {
        self.reset_target = Some(new_target);
    }

    pub fn advance(&mut self, callbacks: &mut impl Callbacks) {
        if let Some(reset_target) = self.reset_target.take() {
            self.state.position = reset_target;
            self.callback_state
                .emit(self.target_time.next(), reset_target, callbacks);
        }
        while self.callback_state.can_append(callbacks) {
            // debug!("ES advance");
            let cmd = match self.current_move.as_mut() {
                None => match self.queue.pop_front() {
                    None => return, // Nothing to do
                    Some(m) => {
                        self.current_move = Some(m);
                        self.current_move.as_mut().unwrap()
                    }
                },
                Some(m) => m,
            };

            // Get next PID tick
            let mut next_time = self.target_time.next();
            while cmd.count != 0 && self.callback_state.can_append(callbacks) {
                // Apply current command up to the next tick
                match self.state.advance(cmd, next_time) {
                    // Command was fully consumed, last_step was left <= next_time
                    AdvanceResult::Consumed => {
                        self.callback_state
                            .emit(next_time, self.state.position, callbacks);
                        self.callback_state.incomplete = true;
                        cmd.count = 0;
                        break;
                    }
                    AdvanceResult::Partial(new_cmd) => {
                        // Force advance to next PID tick
                        self.callback_state
                            .emit(next_time, self.state.position, callbacks);
                        self.callback_state.incomplete = false;
                        next_time = self.target_time.advance();
                        *cmd = new_cmd;
                    }
                    AdvanceResult::FutureMove => {
                        next_time = self.target_time.advance();
                        self.callback_state.incomplete = false;
                    }
                };
            }
            if cmd.count == 0 {
                self.current_move = None
            }
        }
        // debug!("ES advance END");
    }

    pub fn queue_move(&mut self, interval: u32, count: u16, add: i16) -> bool {
        let cmd = Move {
            interval,
            count,
            add,
            direction: self.next_direction,
        };
        // debug!("ES queue_move {}", cmd);
        if self.queue.push_back(cmd).is_err() {
            return false;
        }
        true
    }

    pub fn set_next_dir(&mut self, dir: Direction) {
        self.next_direction = dir;
    }

    pub fn last_step(&self) -> Instant {
        self.state.last_step
    }

    pub fn has_moves(&self) -> bool {
        self.current_move.is_some() || !self.queue.is_empty()
    }

    pub fn move_count(&self) -> usize {
        self.queue.capacity()
    }

    pub fn get_position(&self) -> i32 {
        self.state.position as i32
    }

    pub fn get_commanded_position(&self) -> i32 {
        self.state.position as i32
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        let cmd = if enabled {
            TMCCommand::Enable
        } else {
            TMCCommand::Disable
        };
        debug!("ES set_enabled {}", cmd);
        TMC_CMD.sender().try_send(cmd).ok();
    }
}
