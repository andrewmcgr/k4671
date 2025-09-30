#![no_std]

use core::mem::MaybeUninit;
#[unsafe(no_mangle)]
#[unsafe(link_section = ".uninit")]
static mut BOOTMAGIC: MaybeUninit<u32> = MaybeUninit::uninit();

use core::{arch::asm, ptr};
use defmt::*;

const DFU_BOOT_KEY: u32 = 0xDEADBEEF;
const BOOTLOADER_ST_ADDR: u32 = 0x1fff_0000;

pub fn maybe_enter_dfu() {
    // Safety: This uses memory initialised before the last reset.
    // It either does nothing, or jumps to the bootloader and never returns, or resets the chip.
    unsafe {
        #[allow(static_mut_refs)]
        let bm = crate::BOOTMAGIC.assume_init_mut();
        info!("Bootmagic {:x}", *bm);
        if *bm == !DFU_BOOT_KEY {
            *bm = 0;
            asm!("nop");
            cortex_m::peripheral::SCB::sys_reset();
        }
        if *bm == DFU_BOOT_KEY {
            *bm = !DFU_BOOT_KEY;
            info!("Entering DFU mode");
            let initial_sp = ptr::read(BOOTLOADER_ST_ADDR as *const u32);
            let start_addr = ptr::read((BOOTLOADER_ST_ADDR + 4) as *const u32);
            asm!("mov sp, {0}\nbx {1}", in(reg) initial_sp, in(reg) start_addr);
            core::hint::unreachable_unchecked()
        }
    }
}

pub fn enter_dfu_mode() {
    unsafe {
        #[allow(static_mut_refs)]
        let bm = crate::BOOTMAGIC.assume_init_mut();
        *bm = DFU_BOOT_KEY;
        info!("Bootmagic written {:x}", *bm);
        cortex_m::peripheral::SCB::sys_reset();
    }
}
