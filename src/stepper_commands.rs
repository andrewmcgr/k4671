use crate::LED_STATE;
use crate::LedState::{Connected, Enabled};
use crate::State;
use crate::commands::{Clock32, Clock64, CLOCKS_PER_TICK};
use crate::stepper::Direction;
use anchor::*;
use core::ops::{Deref, DerefMut};
use defmt::*;
use embassy_time::Instant;

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
        info!("Reset step clock {} {}", oid, clock);
        context.steppers[*i].lock(|s| {
            s.borrow_mut()
                .deref_mut()
                .reset_clock(Instant::from(Clock64::from(clock)));
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
        let pos = context.steppers[*i].lock(|s| s.borrow().deref().get_commanded_position());
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
    context.steppers[0].lock(|s| {
        s.borrow_mut().deref_mut().stepper_enable_oid = Some(oid);
    });
    context.steppers_by_enable_oid.insert(oid, 0).ok();
}

#[klipper_command]
pub fn queue_digital_out(context: &mut State, oid: u8, _clock: u32, on_ticks: u32) {
    if let Some(i) = context.steppers_by_enable_oid.get(&oid) {
        info!("Queue digital out {} {} {}", oid, _clock, on_ticks);
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
        info!("Update digital out {} {}", oid, value);
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
pub fn stepper_stop_on_trigger(context: &mut State, oid: u8, trsync_oid: u8) {
    info!("Stepper stop on trigger {} {}", oid, trsync_oid);
    for t in context.trsync.iter() {
        t.lock(|t| {
            let mut t = t.borrow_mut();
            let t = t.deref_mut();
            if t.oid == Some(trsync_oid) {
                info!("Stepper {} registered for TrSync {}", oid, trsync_oid);
                t.stepper_oids.push(oid).ok();
                return;
            }
        });
    }
}

#[klipper_command]
pub fn config_trsync(context: &mut State, oid: u8) {
    info!("Config trsync {}", oid);
    for t in context.trsync.iter() {
        if t.lock(|t| -> bool {
            let mut t = t.borrow_mut();
            let t = t.deref_mut();
            if t.oid.is_none() {
                t.oid = Some(oid);
                info!("TrSync allocated for {}", oid);
                return true;
            }
            false
        }) {
            return;
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
    for t in context.trsync.iter() {
        if t.lock(|t| -> bool {
            let mut t = t.borrow_mut();
            let t = t.deref_mut();
            if t.oid == Some(oid) {
                info!("TrSync starting for {}", oid);
                t.report_clock = if report_clock != 0 {
                    Some(Instant::from(Clock64::from(report_clock)))
                } else {
                    None
                };
                t.report_ticks = Some(report_ticks / CLOCKS_PER_TICK);
                t.trigger_reason = 0;
                t.can_trigger = true;
                t.expire_reason = expire_reason;
                trsync_report(
                    oid,
                    if t.can_trigger { 1 } else { 0 },
                    t.trigger_reason,
                    Clock32::from(report_clock),
                );
                crate::TRSYNC_WATCH.dyn_sender().send(1);
                return true;
            }
            false
        }) {
            return;
        };
    }
}

#[klipper_command]
pub fn trsync_set_timeout(context: &mut State, oid: u8, clock: u32) {
    info!("TrSync set timeout {} {}", oid, clock);
    for t in context.trsync.iter() {
        if t.lock(|t| -> bool {
            let mut t = t.borrow_mut();
            let t = t.deref_mut();
            if t.oid == Some(oid) {
                t.timeout_clock = Some(Instant::from(Clock64::from(clock)));
                return true;
            }
            false
        }) {
            return;
        };
    }
}

#[klipper_command]
pub fn trsync_trigger(context: &mut State, oid: u8, reason: u8) {
    info!("TrSync trigger {} {}", oid, reason);

    for t in context.trsync.iter() {
        if t.lock(|t| -> bool {
            let mut t = t.borrow_mut();
            let t = t.deref_mut();
            info!("TrSync check oid {:?} for {}", t.oid, oid);
            if t.oid == Some(oid) {
                for i in t.stepper_oids.drain(..) {
                    if let Some(si) = context.steppers_by_oid.get(&i) {
                        info!("Stopping stepper {} for TrSync {}", i, oid);
                        context.steppers[*si].lock(|s| {
                            let mut s = s.borrow_mut();
                            let s = s.deref_mut();
                            s.stop();
                        });
                    }
                }
                if t.can_trigger {
                    t.trigger_reason = reason;
                    t.can_trigger = false;
                }
                t.timeout_clock = None;
                t.report_clock = None;
                t.report_ticks = None;
                trsync_report(oid, 0, reason, Clock32::from(0));
                return true;
            }
            false
        }) {
            return;
        };
    }
}

pub fn trsync_report(oid: u8, can_trigger: u8, trigger_reason: u8, clock: Clock32) {
    info!(
        "TrSync report {} {} {} {}",
        oid, can_trigger, trigger_reason, clock
    );
    klipper_reply!(
        trsync_state,
        oid: u8,
        can_trigger: u8,
        trigger_reason: u8,
        clock: u32 = u32::from(clock)
    );
}
