use anchor::*;
use defmt::*;
use crate::State;

#[klipper_command]
pub fn config_stepper(
    context: &mut State,
    oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    context.stepper_oid = Some(oid);
}

#[klipper_command]
pub fn queue_step(context: &mut State, oid: u8, interval: u32, count: u16, add: i16) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context.stepper.queue_move(interval, count, add);
}

#[klipper_command]
pub fn set_next_step_dir(context: &mut State, oid: u8, dir: u8) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context.stepper.set_next_dir(if dir == 1 {
        Direction::Forward
    } else {
        Direction::Backward
    });
}

#[klipper_command]
pub fn reset_step_clock(context: &mut State, oid: u8, clock: u32) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    context.stepper.reset_clock(clock);
}

#[klipper_command]
pub fn stepper_get_position(context: &mut State, oid: u8) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    let pos = context
        .interfaces
        .pid_last_measured_position
        .load(portable_atomic::Ordering::SeqCst);
    klipper_reply!(stepper_position, oid: u8, pos: i32)
}

#[klipper_command]
pub fn stepper_get_commanded_position(context: &mut State, oid: u8) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    let pos = context
        .interfaces
        .pid_last_commanded_position
        .load(portable_atomic::Ordering::SeqCst);
    klipper_reply!(stepper_commanded_position, oid: u8, pos: i32)
}

#[klipper_command]
pub fn stepper_stop_on_trigger(context: &mut State, oid: u8, _trsync_oid: u8) {
    if context.stepper_oid.map_or(false, |o| o != oid) {
        return;
    }
    klipper_shutdown!("trsync not supported", Clock::clock32());
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
    if pin != Pins::Enable.into() {
        return;
    }
    context.stepper_enable_oid = Some(oid);
}

#[klipper_command]
pub fn queue_digital_out(context: &mut State, oid: u8, _clock: u32, on_ticks: u32) {
    if Some(oid) != context.stepper_enable_oid {
        return;
    }
    let enable = on_ticks != 0;
    if !enable {
        context.stepper.reset_target(0);
    }
    context
        .interfaces
        .pid_set_enable
        .store(enable, portable_atomic::Ordering::SeqCst);
}

#[klipper_command]
pub fn update_digital_out(context: &mut State, oid: u8, value: u8) {
    if Some(oid) != context.stepper_enable_oid {
        return;
    }
    let enable = value != 0;
    if !enable {
        context.stepper.reset_target(0);
    }
    context
        .interfaces
        .pid_set_enable
        .store(enable, portable_atomic::Ordering::SeqCst);
}

klipper_enumeration! {
    #[klipper_enumeration(name = "pin", rename_all="snake_case")]
    enum Pins {
        Step,
        Dir,
        Enable,
        Endstop,
    }
}