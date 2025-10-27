use crate::{stepper_commands::Pins, EmulatedStepper, TrSync};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::CriticalSectionMutex as CS;
use heapless::index_map::FnvIndexMap as ObjMap;


pub type KlipperObjectMap<'a> = ObjMap<u8, KlipperObject<'a>, 16>;


#[derive(Debug, defmt::Format, Default)]
pub enum KlipperObject<'a> {
    #[default]
    None,
    Stepper(&'a CS<RefCell<EmulatedStepper>>),
    TrSync(&'a CS<RefCell<TrSync>>),
    EndstopPin(&'a CS<RefCell<Pins>>),
}

impl<'a> KlipperObject<'a> {
    pub fn as_stepper(&self) -> Option<&'a CS<RefCell<EmulatedStepper>>> {
        match self {
            KlipperObject::Stepper(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_trsync(&self) -> Option<&'a CS<RefCell<TrSync>>> {
        match self {
            KlipperObject::TrSync(t) => Some(t),
            _ => None,
        }
    }

    pub fn as_endstop_pin(&self) -> Option<&'a CS<RefCell<Pins>>> {
        match self {
            KlipperObject::EndstopPin(p) => Some(p),
            _ => None,
        }
    }
}