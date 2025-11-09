use crate::LedState::{Connected, Enabled};
use crate::State;
use crate::commands::{clock32_to_instant, clock32_to_ticks, clocki16_to_ticks};
use crate::leds::LED_STATE;
use crate::stepper::Direction;
use embassy_time::Duration;

use anchor::*;
use defmt::*;

#[klipper_command]
pub fn config_stepper(
    context: &mut State,
    oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    info!(
        "Config Stepper {} {} {} {} {}",
        oid, _step_pin, _dir_pin, _invert_step, _step_pulse_ticks
    );

    for i in 0..context.steppers.len() {
        if context.steppers[i].stepper_oid.is_some() {
            continue;
        } else {
            context.steppers_by_oid.insert(oid, i).ok();
            break;
        }
    }
}

static STEP_HORIZON: Duration = Duration::from_millis(1);

#[klipper_command]
pub fn queue_step(context: &mut State, oid: u8, interval: u32, count: u16, add: i16) {
    let mut ticks: u32 = clock32_to_ticks(interval);
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        if count == 1 {
            let last_step = context.steppers[*i].last_step();
            let next_step = clock32_to_instant(interval);
            let diff = next_step.saturating_duration_since(last_step);
            info!("Far future step diff {:?} {:?} {:?}", last_step, next_step, diff);
            if diff > STEP_HORIZON {
                context.steppers[*i].reset_clock(next_step.saturating_sub(STEP_HORIZON));
                ticks = STEP_HORIZON.as_ticks() as u32;
            } else {
                ticks = diff.as_ticks() as u32;
            }
        }
        debug!("queue_step {} ({}) {} {}", ticks, interval, count, add);
        context.steppers[*i].queue_move(
            Duration::from_ticks(ticks as u64),
            count,
            clocki16_to_ticks(add),
        );
    } else {
        warn!("No OID match");
    }
}

#[klipper_command]
pub fn set_next_step_dir(context: &mut State, oid: u8, dir: u8) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Set next step dir {} {}", oid, dir);
        context.steppers[*i].set_next_dir(if dir == 1 {
            debug!("Forward");
            Direction::Forward
        } else {
            debug!("Backward");
            Direction::Backward
        });
    } else {
        warn!("No OID match");
    }
}

// Set an absolute time that the next step command will be relative to.
#[klipper_command]
pub fn reset_step_clock(context: &mut State, oid: u8, clock: u32) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        info!("Reset step clock {} {}", oid, clock);
        context.steppers[*i].reset_clock(clock32_to_instant(clock));
    } else {
        warn!("No OID match");
    }
}

#[klipper_command]
pub fn stepper_get_position(context: &mut State, oid: u8) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Stepper get position {}", oid);
        let pos = context.steppers[*i].get_position();
        info!("Stepper position responds {}", pos);
        klipper_reply!(stepper_position, oid: u8, pos: i32)
    } else {
        warn!("No OID match");
        return;
    }
}

#[klipper_command]
pub fn stepper_get_commanded_position(context: &mut State, oid: u8) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Stepper get commanded position {}", oid);
        let pos = context.steppers[*i].get_commanded_position();
        info!("Stepper commanded position responds {}", pos);
        klipper_reply!(stepper_commanded_position, oid: u8, pos: i32)
    } else {
        warn!("No OID match");
        return;
    }
}

#[klipper_command]
pub fn config_digital_out(
    context: &mut State,
    oid: u8,
    pin: u8,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    let epin = Pins::try_from(pin);
    info!("Config digital out {} {} {}", oid, pin, epin);
    if pin != u8::from(Pins::Enable) {
        return;
    }
    context.steppers[0].stepper_enable_oid = Some(oid);
    context.steppers_by_enable_oid.insert(oid, 0).ok();
}

#[klipper_command]
pub fn queue_digital_out(context: &mut State, oid: u8, _clock: u32, on_ticks: u32) {
    if let Some(i) = context.steppers_by_enable_oid.get(&oid) {
        info!("Queue digital out {} {} {}", oid, _clock, on_ticks);
        let enable = on_ticks != 0;
        context.steppers[*i].reset_target(0);
        context.steppers[*i].set_enabled(enable);
        if enable {
            LED_STATE.signal(Connected);
        } else {
            LED_STATE.signal(Enabled);
        }
    }
}

#[klipper_command]
pub fn update_digital_out(context: &mut State, oid: u8, value: u8) {
    if let Some(i) = context.steppers_by_enable_oid.get(&oid) {
        info!("Update digital out {} {}", oid, value);
        let enable = value != 0;
        if !enable {
            context.steppers[*i].reset_target(0);
            context.steppers[*i].set_enabled(enable);
            LED_STATE.signal(Connected);
        } else {
            LED_STATE.signal(Enabled);
        }
    } else {
        warn!("No OID match");
        return;
    }
}

klipper_enumeration! {
    #[derive(Debug, defmt::Format)]
    #[klipper_enumeration(name = "pin", rename_all="snake_case")]
    enum Pins {
        ChipSelect,
        Step,
        Dir,
        Enable,
        Endstop,
    }
}

#[klipper_command]
pub fn stepper_stop_on_trigger(context: &mut State, oid: u8, trsync_oid: u8) {
    info!("Stepper stop on trigger {} {}", oid, trsync_oid);
    if let Some(i) = context.trsync_by_oid.get(&trsync_oid) {
        if let Some(t) = context.trsync.get_mut(*i) {
            info!("Stepper {} registered for TrSync {}", oid, trsync_oid);
            t.stepper_oids.push(oid).ok();
        }
    }
}

#[klipper_command]
pub fn config_trsync(context: &mut State, oid: u8) {
    info!("Config trsync {}", oid);
    for (i, t) in context.trsync.iter_mut().enumerate() {
        if t.oid.is_none() {
            t.oid = Some(oid);
            context.trsync_by_oid.insert(oid, i).ok();
            info!("TrSync allocated for {}", oid);
            break;
        }
    }
}

#[klipper_command]
pub fn trsync_start(
    context: &mut State,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) {
    info!(
        "TrSync start {} {} {} {}",
        oid, report_clock, report_ticks, expire_reason
    );
    if let Some(i) = context.trsync_by_oid.get(&oid) {
        if let Some(t) = context.trsync.get_mut(*i) {
            info!("TrSync starting for {}", oid);
            t.report_ticks = Some(Duration::from_ticks(clock32_to_ticks(report_ticks) as u64));
            t.report_clock = if report_ticks != 0 {
                Some(clock32_to_instant(report_clock))
            } else {
                None
            };
            t.trigger_reason = 0;
            t.can_trigger = true;
            t.expire_reason = expire_reason;
            t.timeout_clock = None;
            if report_ticks != 0 {
                crate::TRSYNC_WATCH.dyn_sender().send(1);
            }
        };
    }
}

#[klipper_command]
pub fn trsync_set_timeout(context: &mut State, oid: u8, clock: u32) {
    info!("TrSync set timeout {} {}", oid, clock);
    if let Some(i) = context.trsync_by_oid.get(&oid) {
        if let Some(t) = context.trsync.get_mut(*i) {
            t.timeout_clock = Some(clock32_to_instant(clock));
            crate::TRSYNC_WATCH.dyn_sender().send(clock);
        }
    }
}

#[klipper_command]
pub fn trsync_trigger(context: &mut State, oid: u8, reason: u8) {
    info!("TrSync trigger {} {}", oid, reason);

    if let Some(i) = context.trsync_by_oid.get(&oid) {
        if let Some(t) = context.trsync.get_mut(*i) {
            for i in t.stepper_oids.drain(..) {
                if let Some(si) = context.steppers_by_oid.get(&i) {
                    info!("Stopping stepper {} for TrSync {}", i, oid);
                    context.steppers[*si].stop();
                }
            }
            if t.can_trigger {
                t.trigger_reason = reason;
                t.can_trigger = false;
            }
            t.timeout_clock = None;
            t.report_clock = None;
            t.report_ticks = None;
            trsync_report(oid, 0, reason, 0);
            crate::TRSYNC_WATCH.dyn_sender().send(0);
        }
    }
}

pub fn trsync_report(oid: u8, can_trigger: u8, trigger_reason: u8, clock: u32) {
    info!(
        "TrSync report {} {} {} {}",
        oid, can_trigger, trigger_reason, clock
    );
    klipper_reply!(
        trsync_state,
        oid: u8,
        can_trigger: u8,
        trigger_reason: u8,
        clock: u32
    );
}
