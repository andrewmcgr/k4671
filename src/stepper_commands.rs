use crate::LED_STATE;
use crate::LedState::{Connected, Enabled};
use crate::State;
use crate::clock32_to_64;
use crate::stepper::Direction;
use core::ops::{Deref, DerefMut};
use embassy_time::Instant;

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
    debug!(
        "Config Stepper {} {} {} {} {}",
        oid, _step_pin, _dir_pin, _invert_step, _step_pulse_ticks
    );

    for i in 0..context.steppers.len() {
        if context.steppers[i].lock(|s| s.borrow().deref().stepper_oid.is_some()) {
            continue;
        } else {
            context.steppers_by_oid.insert(oid, i).ok();
            break;
        }
    }
}

#[klipper_command]
pub fn queue_step(context: &mut State, oid: u8, interval: u32, count: u16, add: i16) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("queue_step {} {} {}", interval, count, add);
        context.steppers[*i].lock(|s| s.borrow_mut().deref_mut().queue_move(interval, count, add));
    } else {
        warn!("No OID match");
    }
}

#[klipper_command]
pub fn set_next_step_dir(context: &mut State, oid: u8, dir: u8) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Set next step dir {} {}", oid, dir);
        context.steppers[*i].lock(|s| {
            s.borrow_mut().deref_mut().set_next_dir(if dir == 1 {
                debug!("Forward");
                Direction::Forward
            } else {
                debug!("Backward");
                Direction::Backward
            })
        });
    } else {
        warn!("No OID match");
    }
}

// Set an absolute time that the next step command will be relative to.
#[klipper_command]
pub fn reset_step_clock(context: &mut State, oid: u8, clock: u32) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Reset step clock {} {}", oid, clock);
        context.steppers[*i].lock(|s| {
            s.borrow_mut().deref_mut().reset_clock(Instant::from_ticks(
                Instant::now().as_ticks() & (0xffff_ffff << 32) & (clock as u64),
            ));
        });
    } else {
        warn!("No OID match");
    }
}

#[klipper_command]
pub fn stepper_get_position(context: &mut State, oid: u8) {
    if let Some(i) = context.steppers_by_oid.get(&oid) {
        debug!("Stepper get position {}", oid);
        let pos = context.steppers[*i].lock(|s| s.borrow().deref().get_position());
        debug!("Stepper position responds {}", pos);
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
        let pos = context.steppers[*i].lock(|s| s.borrow().deref().get_commanded_position());
        debug!("Stepper commanded position responds {}", pos);
        klipper_reply!(stepper_commanded_position, oid: u8, pos: i32)
    } else {
        warn!("No OID match");
        return;
    }
}

#[klipper_command]
pub fn stepper_stop_on_trigger(context: &mut State, oid: u8, _trsync_oid: u8) {
    if let Some(_i) = context.steppers_by_oid.get(&oid) {
        debug!("Stepper stop on trigger {}", oid);
        for i in 0..context.trsync.stepper_oids.len() {
            if context.trsync.stepper_oids[i] == Some(oid) {
                continue;
            }
            if context.trsync.stepper_oids[i].is_none() {
                context.trsync.stepper_oids[i] = Some(oid);
                return;
            }
        }
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
    debug!("Config digital out {} {} {}", oid, pin, epin);
    if pin != u8::from(Pins::Enable) {
        return;
    }
    context.steppers[0].lock(|s| {
        s.borrow_mut().deref_mut().stepper_enable_oid = Some(oid);
    });
    context.steppers_by_enable_oid.insert(oid, 0).ok();
}

#[klipper_command]
pub fn queue_digital_out(context: &mut State, oid: u8, _clock: u32, on_ticks: u32) {
    if let Some(i) = context.steppers_by_enable_oid.get(&oid) {
        debug!("Queue digital out {} {} {}", oid, _clock, on_ticks);
        let enable = on_ticks != 0;
        context.steppers[*i].lock(|s| {
            let mut sis = s.borrow_mut();
            let sis = sis.deref_mut();
            sis.reset_target(0);
            sis.set_enabled(enable);
        });
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
        debug!("Update digital out {} {}", oid, value);
        let enable = value != 0;
        if !enable {
            context.steppers[*i].lock(|s| {
                let mut sis = s.borrow_mut();
                let sis = sis.deref_mut();
                sis.reset_target(0);
                sis.set_enabled(enable);
            });
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
pub fn config_trsync(context: &mut State, oid: u8) {
    context.trsync.oid = Some(oid);
}

#[klipper_command]
pub fn trsync_start(
    context: &mut State,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) {
    let trsync = &mut context.trsync;
    if trsync.oid != Some(oid) {
        warn!("No OID match");
        return;
    }
    trsync.report_clock = if report_clock != 0 {
        Some(clock32_to_64(report_clock))
    } else {
        None
    };
    trsync.report_ticks = Some(report_ticks);
    trsync.trigger_reason = 0;
    trsync.can_trigger = true;
    trsync.expire_reason = expire_reason;
}

#[klipper_command]
pub fn trsync_set_timeout(context: &mut State, oid: u8, clock: u32) {
    let trsync = &mut context.trsync;
    if trsync.oid != Some(oid) {
        warn!("No OID match");
        return;
    }
    trsync.timeout_clock = Some(clock32_to_64(clock));
}

#[klipper_command]
pub fn trsync_trigger(context: &mut State, oid: u8, reason: u8) {
    let trsync = &mut context.trsync;
    if trsync.oid != Some(oid) {
        warn!("No OID match");
        return;
    }
    for i in 0..trsync.stepper_oids.len() {
        context.steppers[i].lock(|s| {
            let mut sis = s.borrow_mut();
            let sis = sis.deref_mut();
            sis.stop();
        });
    }
    trsync.trigger_reason = reason;
    trsync.can_trigger = false;
    trsync.timeout_clock = None;
    trsync.report_clock = None;
    trsync_report(oid, 0, reason, 0);
}

pub fn trsync_report(oid: u8, can_trigger: u8, reason: u8, clock: u32) {
    klipper_reply!(
        trsync_state,
        oid: u8,
        can_trigger: u8,
        trigger_reason: u8 = reason,
        clock: u32
    );
}
