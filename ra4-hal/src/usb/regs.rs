//! Low-level USBFS register helpers.

use usb_device::{UsbDirection, endpoint::EndpointType};

use crate::pac::{self, usbfs::regs::*};

use super::driver::{set_bempenb_shadow, set_brdyenb_shadow, set_nrdyenb_shadow};
use super::types::PipeBinding;

pub(crate) const USB_DPRPU: u16 = 0x0010;
pub(crate) const USB_ISEL: u16 = 0x0020;
pub(crate) const USB_MBW_8: u16 = 0x0000;
pub(crate) const USB_MBW_16: u16 = 0x0400;
pub(crate) const USB_RCNT: u16 = 0x8000;
pub(crate) const USB_BVAL: u16 = 0x8000;
pub(crate) const USB_BCLR: u16 = 0x4000;
pub(crate) const USB_FRDY: u16 = 0x2000;
pub(crate) const USB_DTLN: u16 = 0x0fff;
pub(crate) const USB_BRDY0: u16 = 0x0001;
pub(crate) const USB_NRDY0: u16 = 0x0001;
pub(crate) const USB_BEMP0: u16 = 0x0001;
pub(crate) const USB_PIPECFG_BULK: u16 = 0x4000;
pub(crate) const USB_PIPECFG_INTERRUPT: u16 = 0x8000;
pub(crate) const USB_PIPECFG_DBLBON: u16 = 0x0200;
pub(crate) const USB_PIPECFG_BFREON: u16 = 0x0400;
pub(crate) const USB_PIPECFG_SHTNAK: u16 = 0x0080;
pub(crate) const USB_PIPECFG_DIR_IN: u16 = 0x0010;
pub(crate) const USB_SQCLR: u16 = 0x0100;
pub(crate) const USB_SQSET: u16 = 0x0080;
pub(crate) const USB_ACLRM: u16 = 0x0200;
pub(crate) const USB_CSCLR: u16 = 0x2000;
pub(crate) const USB_CCPL: u16 = 0x0004;
pub(crate) const USB_PID_MASK: u16 = 0x0003;
pub(crate) const USB_PID_STALL: u16 = 0x0002;
pub(crate) const USB_PID_BUF: u16 = 0x0001;
pub(crate) const USB_PID_NAK: u16 = 0x0000;
pub(crate) const USB_PBUSY: u16 = 0x0020;
pub(crate) const USB_RESM: u16 = 0x4000;
pub(crate) const USB_SOFR: u16 = 0x2000;
pub(crate) const USB_DVST: u16 = 0x1000;
pub(crate) const USB_CTRT: u16 = 0x0800;
pub(crate) const USB_VALID: u16 = 0x0008;
pub(crate) const USB_DVSQ: u16 = 0x0070;
pub(crate) const USB_DS_DFLT: u16 = 0x0010;
pub(crate) const USB_CTSQ: u16 = 0x0007;
pub(crate) const USB_CS_RDDS: u16 = 0x0001;
pub(crate) const USB_CS_WRDS: u16 = 0x0003;
pub(crate) const USB_CS_WRND: u16 = 0x0005;
pub(crate) const USB_CS_RDSS: u16 = 0x0002;
pub(crate) const BRDY_BEMP_MASK: u16 = 0x03ff;
pub(crate) const CFIFO_READY_SPINS: usize = 10_000;
pub(crate) const PIPE0_READY_ATTEMPTS: usize = 8;

#[inline(always)]
pub(crate) fn usbfs() -> pac::usbfs::Usbfs {
    pac::USBFS
}

pub(crate) fn select_pipe0(regs: pac::usbfs::Usbfs, in_direction: bool) {
    let mut value = USB_MBW_8;
    if in_direction {
        value |= USB_ISEL;
    }
    regs.cfifosel().write_value(Cfifosel(value));

    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.cfifosel().read().0;
        let expected = value & (USB_ISEL | USB_RCNT);
        if (fifosel & (USB_ISEL | USB_RCNT)) == expected {
            return;
        }
    }
}

pub(crate) fn select_cfifo(
    regs: pac::usbfs::Usbfs,
    pipe: u8,
    in_direction: bool,
    read_count: bool,
) {
    let mut value = USB_MBW_16 | pipe as u16;
    if in_direction {
        value |= USB_ISEL;
    }
    if read_count {
        value |= USB_RCNT;
    }
    regs.cfifosel().write_value(Cfifosel(value));

    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.cfifosel().read().0;
        let expected = value & (0x000f | USB_ISEL | USB_RCNT);
        if (fifosel & (0x000f | USB_ISEL | USB_RCNT)) == expected {
            return;
        }
    }
}

pub(crate) fn select_d0fifo(regs: pac::usbfs::Usbfs, pipe: u8) {
    let expected = USB_MBW_16 | pipe as u16;
    regs.d0fifosel().write_value(D0fifosel(expected));
    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.d0fifosel().read().0;
        if (fifosel & (0x000f | USB_MBW_16)) == expected {
            return;
        }
    }
}

pub(crate) fn cfifo_ready(regs: pac::usbfs::Usbfs) -> bool {
    for _ in 0..CFIFO_READY_SPINS {
        if (regs.cfifoctr().read().0 & USB_FRDY) != 0 {
            return true;
        }

        let _ = regs.syscfg().read().0;
        let _ = regs.syssts0().read().0;
        cortex_m::asm::delay(480);
    }
    false
}

pub(crate) fn select_pipe0_ready(regs: pac::usbfs::Usbfs, in_direction: bool) -> bool {
    for _ in 0..PIPE0_READY_ATTEMPTS {
        select_pipe0(regs, in_direction);
        if cfifo_ready(regs) {
            return true;
        }
    }

    false
}

pub(crate) fn d0fifo_ready(regs: pac::usbfs::Usbfs) -> bool {
    for _ in 0..CFIFO_READY_SPINS {
        if (regs.d0fifoctr().read().0 & USB_FRDY) != 0 {
            return true;
        }

        let _ = regs.syscfg().read().0;
        let _ = regs.syssts0().read().0;
        cortex_m::asm::delay(480);
    }
    false
}

pub(crate) fn write_cfifo(regs: pac::usbfs::Usbfs, buf: &[u8]) {
    let ptr = regs.cfifo().as_ptr() as *mut u8;
    for &byte in buf {
        unsafe { ptr.write_volatile(byte) };
    }
}

pub(crate) fn read_cfifo(regs: pac::usbfs::Usbfs, buf: &mut [u8]) {
    if buf.is_empty() {
        return;
    }

    let ptr16 = regs.cfifo().as_ptr() as *const u16;
    let mut chunks = buf.chunks_exact_mut(2);
    for chunk in &mut chunks {
        let word = unsafe { ptr16.read_volatile() }.to_le_bytes();
        chunk.copy_from_slice(&word);
    }

    let rem = chunks.into_remainder();
    if let Some(last) = rem.first_mut() {
        let word = unsafe { ptr16.read_volatile() }.to_le_bytes();
        *last = word[0];
    }
}

pub(crate) fn write_d0fifo(regs: pac::usbfs::Usbfs, buf: &[u8]) {
    if buf.is_empty() {
        return;
    }

    let fifosel = regs.d0fifosel().read().0;
    regs.d0fifosel()
        .write_value(D0fifosel((fifosel & !0x0c00) | USB_MBW_8));

    let ptr8 = regs.d0fifo().as_ptr() as *mut u8;
    for &byte in buf {
        unsafe { ptr8.write_volatile(byte) };
    }

    regs.d0fifosel()
        .write_value(D0fifosel((fifosel & !0x0c00) | USB_MBW_16));
}

pub(crate) fn clear_intsts0(regs: pac::usbfs::Usbfs, mask: u16) {
    regs.intsts0().write_value(Intsts0(!mask));
}

pub(crate) fn clear_brdy0(regs: pac::usbfs::Usbfs) {
    regs.brdysts()
        .write_value(Brdysts((!USB_BRDY0) & BRDY_BEMP_MASK));
}

pub(crate) fn clear_nrdy0(regs: pac::usbfs::Usbfs) {
    regs.nrdysts()
        .write_value(Nrdysts((!USB_NRDY0) & BRDY_BEMP_MASK));
}

pub(crate) fn clear_bemp0(regs: pac::usbfs::Usbfs) {
    regs.bempsts()
        .write_value(Bempsts((!USB_BEMP0) & BRDY_BEMP_MASK));
}

pub(crate) fn clear_brdy(regs: pac::usbfs::Usbfs, pipe: u8) {
    let bit = 1u16 << pipe;
    regs.brdysts().write_value(Brdysts((!bit) & BRDY_BEMP_MASK));
}

pub(crate) fn clear_nrdy(regs: pac::usbfs::Usbfs, pipe: u8) {
    let bit = 1u16 << pipe;
    regs.nrdysts().write_value(Nrdysts((!bit) & BRDY_BEMP_MASK));
}

pub(crate) fn clear_bemp(regs: pac::usbfs::Usbfs, pipe: u8) {
    let bit = 1u16 << pipe;
    regs.bempsts().write_value(Bempsts((!bit) & BRDY_BEMP_MASK));
}

pub(crate) fn write_brdyenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_brdyenb_shadow(mask);
    regs.brdyenb().write_value(Brdyenb(mask));
}

pub(crate) fn write_nrdyenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_nrdyenb_shadow(mask);
    regs.nrdyenb().write_value(Nrdyenb(mask));
}

pub(crate) fn write_bempenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_bempenb_shadow(mask);
    regs.bempenb().write_value(Bempenb(mask));
}

pub(crate) fn update_brdyenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_brdyenb(regs, shadow, next);
}

pub(crate) fn update_nrdyenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_nrdyenb(regs, shadow, next);
}

pub(crate) fn update_bempenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_bempenb(regs, shadow, next);
}

pub(crate) fn read_pipectr(pipe: u8) -> u16 {
    let regs = usbfs();
    match pipe {
        1..=5 => regs.pipectr((pipe - 1) as usize).read().0,
        6..=9 => regs.pipectr2((pipe - 6) as usize).read().0,
        _ => 0,
    }
}

pub(crate) fn write_pipectr(pipe: u8, value: u16) {
    write_pipectr_reason(pipe, value, 0);
}

pub(crate) fn write_pipectr_reason(pipe: u8, value: u16, reason: u8) {
    let regs = usbfs();
    match pipe {
        1..=5 => regs
            .pipectr((pipe - 1) as usize)
            .write_value(Pipectr(value)),
        6..=9 => regs
            .pipectr2((pipe - 6) as usize)
            .write_value(Pipectr2(value)),
        _ => return,
    }
    let _ = reason;
}

pub(crate) fn set_pipectr_bits(pipe: u8, bits: u16) {
    let current = read_pipectr(pipe);
    write_pipectr(pipe, current | bits);
}

pub(crate) fn clear_pipectr_bits(pipe: u8, bits: u16) {
    let current = read_pipectr(pipe);
    write_pipectr(pipe, current & !bits);
}

pub(crate) fn pulse_pipectr_bits(pipe: u8, bits: u16) {
    set_pipectr_bits(pipe, bits);
    clear_pipectr_bits(pipe, bits);
}

pub(crate) fn set_pipe_nak_wait(pipe: u8) {
    let current = read_pipectr(pipe);
    // Match Renesas usb_cstd_set_nak(): clear BUF first, then wait for PBUSY to drop.
    write_pipectr_reason(pipe, current & !USB_PID_BUF, 1);

    for _ in 0..CFIFO_READY_SPINS {
        let current = read_pipectr(pipe);
        if (current & USB_PBUSY) == 0 {
            return;
        }
    }
}

pub(crate) fn configure_pipe(
    regs: pac::usbfs::Usbfs,
    brdyenb_shadow: &mut u16,
    nrdyenb_shadow: &mut u16,
    bempenb_shadow: &mut u16,
    pipe: u8,
    binding: PipeBinding,
) {
    let pipe_type = match binding.ep_type {
        EndpointType::Bulk => USB_PIPECFG_BULK | USB_PIPECFG_DBLBON,
        EndpointType::Interrupt => USB_PIPECFG_INTERRUPT,
        _ => return,
    };
    let pipe_dir = if binding.ep_addr.direction() == UsbDirection::In {
        USB_PIPECFG_DIR_IN
    } else {
        0
    };

    set_pipe_nak_wait(pipe);
    regs.pipesel().write_value(Pipesel(pipe as u16));
    let bf_re = if binding.ep_addr.direction() == UsbDirection::Out {
        0
    } else {
        USB_PIPECFG_BFREON
    };
    let out_rx_mode = if binding.ep_addr.direction() == UsbDirection::Out
        && matches!(binding.ep_type, EndpointType::Bulk)
    {
        USB_PIPECFG_SHTNAK
    } else {
        0
    };
    regs.pipecfg().write_value(Pipecfg(
        bf_re | out_rx_mode | pipe_type | pipe_dir | binding.ep_addr.index() as u16,
    ));
    regs.pipemaxp()
        .write_value(Pipemaxp(binding.max_packet & 0x01ff));
    regs.pipeperi().write_value(Pipeperi(0));
    regs.pipesel().write_value(Pipesel(0));
    write_pipectr_reason(pipe, USB_SQCLR | USB_PID_NAK, 3);
    set_pipectr_bits(pipe, USB_CSCLR);
    pulse_pipectr_bits(pipe, USB_ACLRM);
    clear_brdy(regs, pipe);
    clear_nrdy(regs, pipe);
    clear_bemp(regs, pipe);
    let bit = 1u16 << pipe;
    if binding.ep_addr.direction() == UsbDirection::Out {
        write_brdyenb(regs, brdyenb_shadow, *brdyenb_shadow | bit);
        write_nrdyenb(regs, nrdyenb_shadow, *nrdyenb_shadow | bit);
        write_bempenb(regs, bempenb_shadow, *bempenb_shadow & !bit);
    } else {
        write_brdyenb(regs, brdyenb_shadow, *brdyenb_shadow & !bit);
        write_nrdyenb(regs, nrdyenb_shadow, *nrdyenb_shadow & !bit);
        write_bempenb(regs, bempenb_shadow, *bempenb_shadow & !bit);
    }
}

pub(crate) fn clear_pipe_config(regs: pac::usbfs::Usbfs, pipe: u8) {
    set_pipe_nak_wait(pipe);
    regs.pipesel().write_value(Pipesel(pipe as u16));
    regs.pipecfg().write_value(Pipecfg(0));
    regs.pipemaxp().write_value(Pipemaxp(0));
    regs.pipeperi().write_value(Pipeperi(0));
    regs.pipesel().write_value(Pipesel(0));
    write_pipectr_reason(pipe, USB_SQCLR | USB_PID_NAK, 4);
    set_pipectr_bits(pipe, USB_CSCLR);
    pulse_pipectr_bits(pipe, USB_ACLRM);
    clear_brdy(regs, pipe);
    clear_nrdy(regs, pipe);
    clear_bemp(regs, pipe);
}

pub(crate) fn start_out_receive(
    regs: pac::usbfs::Usbfs,
    brdyenb_shadow: &mut u16,
    nrdyenb_shadow: &mut u16,
    pipe: u8,
    _max_packet: u16,
    _requested_len: u32,
) {
    if !(1..=9).contains(&pipe) {
        return;
    }

    let bit = 1u16 << pipe;
    set_pipe_nak_wait(pipe);
    select_cfifo(regs, pipe, false, false);
    clear_brdy(regs, pipe);
    clear_nrdy(regs, pipe);
    let current = read_pipectr(pipe);
    write_pipectr_reason(pipe, (current & !USB_PID_MASK) | USB_PID_BUF, 5);
    write_brdyenb(regs, brdyenb_shadow, *brdyenb_shadow | bit);
    write_nrdyenb(regs, nrdyenb_shadow, *nrdyenb_shadow | bit);
}
