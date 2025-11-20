use core::hint::*;

use crate::{
    TMC_CMD,
    commands::{clock32_to_instant, duration_to_ticks, instant_to_clock32},
};
use defmt::*;
use embassy_time::Instant;
use heapless::Deque;
use tmc4671::*;

pub type TargetQueue = crate::target_queue::TargetQueue<512>;

#[derive(Debug, defmt::Format, Copy, Clone, Eq, PartialEq, PartialOrd, Ord)]
pub enum Direction {
    Forward,
    Backward,
}

#[derive(Default, Debug, Copy, Clone, defmt::Format)]
pub enum MoveQueueKind {
    #[default]
    Move,
    Enable(bool),
}
impl MoveQueueKind {
    fn into_enable(&self) -> Option<bool> {
        match self {
            MoveQueueKind::Move => None,
            MoveQueueKind::Enable(e) => Some(*e),
        }
    }
}

#[derive(Debug, Copy, Clone, defmt::Format)]
#[repr(C)]
pub struct Move {
    interval: u32,
    count: u16,
    add: i16,
    direction: Direction,
    kind: MoveQueueKind,
}

impl Move {
    fn total_time(&self) -> u32 {
        self.time_after_steps(self.count)
    }

    fn time_after_steps(&self, steps: u16) -> u32 {
        if steps == 0 {
            return 0;
        }
        let base = (steps as u64) * (self.interval as u64);
        let accel = (self.add as i32) * (steps as i32 - 1) * (steps as i32) / 2;
        base.wrapping_add(accel as u64) as u32
    }

    fn steps_before_time(&self, target: u32) -> u16 {
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
            kind: MoveQueueKind::Move,
        }
    }
}

#[derive(Debug)]
struct State {
    last_step: u32,
    position: u32,
}

impl State {
    /// Advances the state by the given move, up to maximum time
    fn advance(&mut self, cmd: &Move, up_to_time: Instant) -> AdvanceResult {
        let next_clock32 = self.last_step.wrapping_add(cmd.interval);
        let next_step = clock32_to_instant(next_clock32);

        if next_step > up_to_time {
            return AdvanceResult::FutureMove;
        }
        debug!(
            "Next step at {}, now is {}, up to {}",
            next_step,
            Instant::now(),
            up_to_time
        );

        self.last_step = next_clock32;

        if let MoveQueueKind::Enable(_) = cmd.kind {
            return AdvanceResult::Consumed;
        }

        // If the next step is within our window, consume one step
        self.step(cmd.direction, 1);
        let cmd = cmd.advance(1);
        if cmd.count == 0 {
            return AdvanceResult::Consumed; // We consumed the entire thing.
        }

        // Now see if we can apply more steps
        let available_time =
            match up_to_time.checked_duration_since(clock32_to_instant(self.last_step)) {
                Some(t) => duration_to_ticks(t),
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
    fn append(&mut self, time: u32, value: u32, enable: Option<bool>);
    fn update_last(&mut self, time: u32, value: u32);
    fn can_append(&self) -> bool;
}

#[derive(Debug)]
pub struct EmulatedStepper<T, const N: usize> {
    pub index: usize,
    queue: heapless::Deque<Move, N>,
    pub target_queue: TargetQueue,
    current_move: Option<Move>,
    next_direction: Direction,
    state: State,
    target_time: T,
    callback_state: CallbackState,
    reset_target: Option<u32>,
    pub stepper_oid: Option<u8>,
    pub stepper_enable_oid: Option<u8>,
    pub enabled: bool,
}

#[derive(Debug)]
struct CallbackState {
    last_append: (u32, u32),
    incomplete: bool,
}

impl CallbackState {
    fn append(
        &mut self,
        next_time: u32,
        position: u32,
        enable: Option<bool>,
        callbacks: &mut impl Callbacks,
    ) {
        debug!(
            "Append callback at {} pos {} enable {:?}",
            next_time, position, enable
        );
        callbacks.append(next_time, position, enable);
        self.last_append = (next_time, position);
    }

    fn update(&mut self, position: u32, callbacks: &mut impl Callbacks) {
        let pos = position;
        if pos != self.last_append.1 {
            callbacks.update_last(self.last_append.0, pos);
        }
    }

    fn emit(
        &mut self,
        next_time: u32,
        position: u32,
        enable: Option<bool>,
        callbacks: &mut impl Callbacks,
    ) {
        debug!(
            "Emit callback at {} pos {} enable {:?} incomplete {}",
            next_time, position, enable, self.incomplete
        );
        if likely(enable.is_some()) || !self.incomplete {
            self.append(next_time, position, enable, callbacks);
        } else {
            self.update(position, callbacks);
        }
    }

    fn can_append(&self, callbacks: &impl Callbacks) -> bool {
        callbacks.can_append()
    }
}

impl<T: tmc4671::TimeIterator, const N: usize> EmulatedStepper<T, N> {
    pub fn new(index: usize, target_time: T) -> Self {
        Self {
            index: index,
            queue: Deque::new(),
            target_queue: TargetQueue::new(),
            stepper_oid: None,
            stepper_enable_oid: None,
            current_move: None,
            next_direction: Direction::Forward,
            state: State {
                last_step: 0,
                position: 0,
            },
            target_time,
            callback_state: CallbackState {
                last_append: (0, 0),
                incomplete: true,
            },
            reset_target: None,
            enabled: false,
        }
    }

    pub fn reset_clock(&mut self, time: u32) {
        debug!("Reset clock to {}", time);
        self.state.last_step = time;
    }

    pub fn current_position(&self) -> i32 {
        self.state.position as i32
    }

    pub fn reset_target(&mut self, new_target: u32) {
        self.reset_target = Some(new_target);
    }

    pub fn advance(&mut self) {
        let callbacks = &mut self.target_queue;
        if let Some(reset_target) = self.reset_target.take() {
            self.state.position = reset_target;
            self.callback_state.emit(
                instant_to_clock32(self.target_time.next()),
                reset_target,
                None,
                callbacks,
            );
        }
        while self.callback_state.can_append(callbacks) {
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
            let mut next_time = self.target_time.advance();
            debug!("Advancing stepper with command {:?} at {}", cmd, next_time);
            while cmd.count != 0 && self.callback_state.can_append(callbacks) {
                // Apply current command up to the next tick
                match self.state.advance(cmd, next_time) {
                    // Command was fully consumed, last_step was left <= next_time
                    AdvanceResult::Consumed => {
                        debug!("Command fully consumed");
                        self.callback_state.emit(
                            instant_to_clock32(next_time),
                            self.state.position,
                            cmd.kind.into_enable(),
                            callbacks,
                        );
                        self.callback_state.incomplete = true;
                        cmd.count = 0;
                        // break;
                    }
                    AdvanceResult::Partial(new_cmd) => {
                        debug!("Command partially consumed, new command {:?}", new_cmd);
                        // Force advance to next PID tick
                        self.callback_state.emit(
                            instant_to_clock32(next_time),
                            self.state.position,
                            None,
                            callbacks,
                        );
                        self.callback_state.incomplete = false;
                        // Can be used on the next iteration, modifies time iterator as well
                        #[expect(unused_assignments)]
                        next_time = self.target_time.advance();
                        *cmd = new_cmd;
                        break;
                    }
                    AdvanceResult::FutureMove => {
                        debug!("Command not yet ready, advancing time");
                        self.callback_state.incomplete = false;
                        return;
                    }
                };
            }
            if cmd.count == 0 {
                self.current_move = None
            }
        }
    }

    pub fn queue_move(&mut self, interval: u32, count: u16, add: i16) -> bool {
        let cmd = Move {
            interval,
            count,
            add,
            direction: self.next_direction,
            kind: MoveQueueKind::Move,
        };
        debug!("ES queue_move {} {}", cmd, self.queue.len());
        if unlikely(self.queue.push_back(cmd).is_err()) {
            warn!("ES queue full");
            return false;
        }
        true
    }

    pub fn stop(&mut self) {
        self.queue.clear();
        self.target_queue.clear();
        self.current_move = None;
        TMC_CMD[self.index].enqueue(TMCCommand::Stop).ok();
    }

    pub fn set_next_dir(&mut self, dir: Direction) {
        self.next_direction = dir;
    }

    pub fn last_step(&self) -> u32 {
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

    pub fn set_enabled(&mut self, interval: u32, enabled: bool) {
        let cmd = Move {
            interval,
            count: 1,
            add: 0,
            direction: self.next_direction,
            kind: MoveQueueKind::Enable(enabled),
        };
        if self.queue.push_back(cmd).is_err() {
            warn!("ES queue full");
        }
    }
}
