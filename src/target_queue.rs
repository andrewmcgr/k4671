use crate::{commands::{clock32_to_instant, instant_to_clock32}, stepper::Callbacks};
use embassy_time::Instant;
use heapless::Deque;


pub struct ControlOutput {
    pub position: i32,
    pub time: Option<Instant>,
    pub position_1: Option<(Instant, i32)>,
    pub position_2: Option<(Instant, i32)>,
}

impl ControlOutput {
    fn single(position: i32, time: Option<Instant>) -> Self {
        Self {
            position,
            time,
            position_1: None,
            position_2: None,
        }
    }
}

#[derive(Debug)]
pub struct TargetQueue<const N: usize> {
    queue: Deque<(u32, u32), N>,
    last_value: u32,
}

impl<const N: usize> Default for TargetQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TargetQueue<N> {
    pub fn new() -> Self {
        Self {
            queue: Deque::new(),
            last_value: 0,
        }
    }

    fn can_append(&self) -> bool {
        !self.queue.is_full()
    }

    pub fn clear(&mut self) {
       self.queue.clear();
    }

    fn append(&mut self, time: u32, value: u32) {
        self.queue.push_back((time, value)).ok();
        self.last_value = value;
    }

    fn update_last(&mut self, time: u32, value: u32) {
        if let Some(v) = self.queue.back_mut() {
            *v = (time, value);
        }
        self.last_value = value;
    }

    pub fn get_for_control(&mut self, time: Instant) -> ControlOutput {
            let last = self.last_value as i32;
            let time = instant_to_clock32(time);

            // Remove from front such that the next item will be read now

            while let Some((t, _)) = self.queue.front() {
                if *t >= time {
                    break;
                }
                self.queue.pop_front();
            }
            if self.queue.is_empty() {
                return ControlOutput::single(last, Some(clock32_to_instant(time)));
            }
            let mut iter = self.queue.iter();
            let v0 = iter.next().copied();
            let (t0, v0) = match v0 {
                Some((t0, v0)) if t0 == time => (t0, v0),
                _ => return ControlOutput::single(last, Some(clock32_to_instant(time))),
            };
            let v1 = iter.next().copied();
            let v2 = iter.next().copied();
            ControlOutput {
                position: v0 as i32,
                time: Some(clock32_to_instant(t0)),
                position_1: v1.map(|(t, v)| (clock32_to_instant(t), v as i32)),
                position_2: v2.map(|(t, v)| (clock32_to_instant(t), v as i32)),
            }
        
    }
}

impl<const N: usize> Callbacks for TargetQueue<N> {
    fn append(&mut self, time: u32, value: u32) {
        TargetQueue::append(self, time, value)
    }

    fn update_last(&mut self, time: u32, value: u32) {
        TargetQueue::update_last(self, time, value)
    }

    fn can_append(&self) -> bool {
        TargetQueue::can_append(self)
    }
}
