//! Low-level USBFS register helpers.

use usb_device::{UsbDirection, endpoint::EndpointType};

use crate::pac::usbfs::vals::{
    CfifoselCurpipe, D0fifoselCurpipe, DcpctrPid, Pipectr2Pid, PipectrPid, Type,
};
use crate::pac::{self, usbfs::regs::*};

use super::driver::{set_bempenb_shadow, set_brdyenb_shadow, set_nrdyenb_shadow};
use super::types::PipeBinding;

pub(super) const BRDY_BEMP_MASK: u16 = 0x03ff;
pub(super) const CFIFO_READY_SPINS: usize = 10_000;
pub(super) const PIPE0_READY_ATTEMPTS: usize = 8;

#[derive(Clone, Copy)]
pub(super) enum PipePid {
    Nak,
    Buf,
}

#[inline(always)]
pub(super) fn usbfs() -> pac::usbfs::Usbfs {
    pac::USBFS
}

pub(super) fn select_pipe0(regs: pac::usbfs::Usbfs, in_direction: bool) {
    let mut value = Cfifosel::default();
    value.set_mbw(false);
    value.set_isel(in_direction);
    value.set_rcnt(false);
    regs.cfifosel().write_value(value);

    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.cfifosel().read();
        if fifosel.isel() == in_direction && !fifosel.rcnt() && !fifosel.mbw() {
            return;
        }
    }
}

pub(super) fn select_cfifo(
    regs: pac::usbfs::Usbfs,
    pipe: u8,
    in_direction: bool,
    read_count: bool,
) {
    let mut value = Cfifosel::default();
    value.set_curpipe(CfifoselCurpipe::from_bits(pipe));
    value.set_mbw(true);
    value.set_isel(in_direction);
    value.set_rcnt(read_count);
    regs.cfifosel().write_value(value);

    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.cfifosel().read();
        if fifosel.curpipe().to_bits() == pipe
            && fifosel.isel() == in_direction
            && fifosel.rcnt() == read_count
            && fifosel.mbw()
        {
            return;
        }
    }
}

pub(super) fn select_d0fifo(regs: pac::usbfs::Usbfs, pipe: u8) {
    let mut expected = D0fifosel::default();
    expected.set_curpipe(D0fifoselCurpipe::from_bits(pipe));
    expected.set_mbw(true);
    regs.d0fifosel().write_value(expected);
    for _ in 0..CFIFO_READY_SPINS {
        let fifosel = regs.d0fifosel().read();
        if fifosel.curpipe().to_bits() == pipe && fifosel.mbw() {
            return;
        }
    }
}

pub(super) fn cfifo_ready(regs: pac::usbfs::Usbfs) -> bool {
    for _ in 0..CFIFO_READY_SPINS {
        if regs.cfifoctr().read().frdy() {
            return true;
        }

        let _ = regs.syscfg().read().0;
        let _ = regs.syssts0().read().0;
        cortex_m::asm::delay(480);
    }
    false
}

pub(super) fn select_pipe0_ready(regs: pac::usbfs::Usbfs, in_direction: bool) -> bool {
    for _ in 0..PIPE0_READY_ATTEMPTS {
        select_pipe0(regs, in_direction);
        if cfifo_ready(regs) {
            return true;
        }
    }

    false
}

pub(super) fn d0fifo_ready(regs: pac::usbfs::Usbfs) -> bool {
    for _ in 0..CFIFO_READY_SPINS {
        if regs.d0fifoctr().read().frdy() {
            return true;
        }

        let _ = regs.syscfg().read().0;
        let _ = regs.syssts0().read().0;
        cortex_m::asm::delay(480);
    }
    false
}

pub(super) fn write_cfifo(regs: pac::usbfs::Usbfs, buf: &[u8]) {
    let ptr = regs.cfifo().as_ptr() as *mut u8;
    for &byte in buf {
        unsafe { ptr.write_volatile(byte) };
    }
}

pub(super) fn read_cfifo(regs: pac::usbfs::Usbfs, buf: &mut [u8]) {
    if buf.is_empty() {
        return;
    }

    if regs.cfifosel().read().mbw() {
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
    } else {
        let ptr8 = regs.cfifo().as_ptr() as *const u8;
        for byte in buf {
            *byte = unsafe { ptr8.read_volatile() };
        }
    }
}

pub(super) fn write_d0fifo(regs: pac::usbfs::Usbfs, buf: &[u8]) {
    if buf.is_empty() {
        return;
    }

    let fifosel = regs.d0fifosel().read().0;
    let mut d0fifosel = D0fifosel(fifosel);
    d0fifosel.set_mbw(false);
    regs.d0fifosel().write_value(d0fifosel);

    let ptr8 = regs.d0fifo().as_ptr() as *mut u8;
    for &byte in buf {
        unsafe { ptr8.write_volatile(byte) };
    }

    let mut d0fifosel = D0fifosel(fifosel);
    d0fifosel.set_mbw(true);
    regs.d0fifosel().write_value(d0fifosel);
}

pub(super) fn clear_d0fifo_buffer(regs: pac::usbfs::Usbfs) {
    let mut ctr = D0fifoctr::default();
    ctr.set_bclr(true);
    regs.d0fifoctr().write_value(ctr);
}

pub(super) fn set_d0fifo_bval(regs: pac::usbfs::Usbfs) {
    let mut ctr = D0fifoctr::default();
    ctr.set_bval(true);
    regs.d0fifoctr().write_value(ctr);
}

pub(super) fn clear_cfifo_buffer(regs: pac::usbfs::Usbfs) {
    let mut ctr = Cfifoctr::default();
    ctr.set_bclr(true);
    regs.cfifoctr().write_value(ctr);
}

pub(super) fn set_cfifo_bval(regs: pac::usbfs::Usbfs) {
    let mut ctr = Cfifoctr::default();
    ctr.set_bval(true);
    regs.cfifoctr().write_value(ctr);
}

pub(super) fn cfifo_dtln(regs: pac::usbfs::Usbfs) -> usize {
    regs.cfifoctr().read().dtln() as usize
}

pub(super) fn clear_valid(regs: pac::usbfs::Usbfs) {
    let mut value = Intsts0(0xffff);
    value.set_valid(false);
    regs.intsts0().write_value(value);
}

pub(super) fn clear_dvst(regs: pac::usbfs::Usbfs) {
    let mut value = Intsts0(0xffff);
    value.set_dvst(false);
    regs.intsts0().write_value(value);
}

pub(super) fn clear_sofr(regs: pac::usbfs::Usbfs) {
    let mut value = Intsts0(0xffff);
    value.set_sofr(false);
    regs.intsts0().write_value(value);
}

pub(super) fn clear_resm(regs: pac::usbfs::Usbfs) {
    let mut value = Intsts0(0xffff);
    value.set_resm(false);
    regs.intsts0().write_value(value);
}

pub(super) fn clear_ctrt(regs: pac::usbfs::Usbfs) {
    let mut value = Intsts0(0xffff);
    value.set_ctrt(false);
    regs.intsts0().write_value(value);
}

pub(super) fn clear_brdy0(regs: pac::usbfs::Usbfs) {
    let mut value = Brdysts(BRDY_BEMP_MASK);
    value.set_brdy(0, false);
    regs.brdysts().write_value(value);
}

pub(super) fn clear_nrdy0(regs: pac::usbfs::Usbfs) {
    let mut value = Nrdysts(BRDY_BEMP_MASK);
    value.set_nrdy(0, false);
    regs.nrdysts().write_value(value);
}

pub(super) fn clear_bemp0(regs: pac::usbfs::Usbfs) {
    let mut value = Bempsts(BRDY_BEMP_MASK);
    value.set_bemp(0, false);
    regs.bempsts().write_value(value);
}

pub(super) fn clear_brdy(regs: pac::usbfs::Usbfs, pipe: u8) {
    let mut value = Brdysts(BRDY_BEMP_MASK);
    value.set_brdy(pipe as usize, false);
    regs.brdysts().write_value(value);
}

pub(super) fn clear_nrdy(regs: pac::usbfs::Usbfs, pipe: u8) {
    let mut value = Nrdysts(BRDY_BEMP_MASK);
    value.set_nrdy(pipe as usize, false);
    regs.nrdysts().write_value(value);
}

pub(super) fn clear_bemp(regs: pac::usbfs::Usbfs, pipe: u8) {
    let mut value = Bempsts(BRDY_BEMP_MASK);
    value.set_bemp(pipe as usize, false);
    regs.bempsts().write_value(value);
}

pub(super) fn write_brdyenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_brdyenb_shadow(mask);
    regs.brdyenb().write_value(Brdyenb(mask));
}

pub(super) fn write_nrdyenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_nrdyenb_shadow(mask);
    regs.nrdyenb().write_value(Nrdyenb(mask));
}

pub(super) fn write_bempenb(regs: pac::usbfs::Usbfs, shadow: &mut u16, mask: u16) {
    *shadow = mask;
    set_bempenb_shadow(mask);
    regs.bempenb().write_value(Bempenb(mask));
}

pub(super) fn update_brdyenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_brdyenb(regs, shadow, next);
}

pub(super) fn update_nrdyenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_nrdyenb(regs, shadow, next);
}

pub(super) fn update_bempenb(
    regs: pac::usbfs::Usbfs,
    shadow: &mut u16,
    set_bits: u16,
    clear_bits: u16,
) {
    let next = (*shadow | set_bits) & !clear_bits;
    write_bempenb(regs, shadow, next);
}

pub(super) fn pipe_busy(pipe: u8) -> bool {
    let regs = usbfs();
    match pipe {
        1..=5 => regs.pipectr((pipe - 1) as usize).read().pbusy(),
        6..=9 => regs.pipectr2((pipe - 6) as usize).read().pbusy(),
        _ => false,
    }
}

pub(super) fn set_pipe_pid(pipe: u8, pid: PipePid) {
    let regs = usbfs();
    match pipe {
        1..=5 => {
            let mut ctr = regs.pipectr((pipe - 1) as usize).read();
            ctr.set_pid(match pid {
                PipePid::Nak => PipectrPid::_00,
                PipePid::Buf => PipectrPid::_01,
            });
            regs.pipectr((pipe - 1) as usize).write_value(ctr);
        }
        6..=9 => {
            let mut ctr = regs.pipectr2((pipe - 6) as usize).read();
            ctr.set_pid(match pid {
                PipePid::Nak => Pipectr2Pid::_00,
                PipePid::Buf => Pipectr2Pid::_01,
            });
            regs.pipectr2((pipe - 6) as usize).write_value(ctr);
        }
        _ => {}
    }
}

pub(super) fn pipe_pid_is_buf(pipe: u8) -> bool {
    let regs = usbfs();
    match pipe {
        1..=5 => regs.pipectr((pipe - 1) as usize).read().pid() == PipectrPid::_01,
        6..=9 => regs.pipectr2((pipe - 6) as usize).read().pid() == Pipectr2Pid::_01,
        _ => false,
    }
}

pub(super) fn reset_pipe_control(pipe: u8) {
    let regs = usbfs();
    match pipe {
        1..=5 => {
            let mut ctr = Pipectr::default();
            ctr.set_sqclr(true);
            ctr.set_pid(PipectrPid::_00);
            regs.pipectr((pipe - 1) as usize).write_value(ctr);
            let mut ctr = regs.pipectr((pipe - 1) as usize).read();
            ctr.set_aclrm(true);
            regs.pipectr((pipe - 1) as usize).write_value(ctr);
            ctr.set_aclrm(false);
            regs.pipectr((pipe - 1) as usize).write_value(ctr);
        }
        6..=9 => {
            let mut ctr = Pipectr2::default();
            ctr.set_sqclr(true);
            ctr.set_pid(Pipectr2Pid::_00);
            regs.pipectr2((pipe - 6) as usize).write_value(ctr);
            let mut ctr = regs.pipectr2((pipe - 6) as usize).read();
            ctr.set_aclrm(true);
            regs.pipectr2((pipe - 6) as usize).write_value(ctr);
            ctr.set_aclrm(false);
            regs.pipectr2((pipe - 6) as usize).write_value(ctr);
        }
        _ => {}
    }
}

pub(super) fn set_dcp_pid(regs: pac::usbfs::Usbfs, pid: DcpctrPid, ccpl: bool) {
    let mut dcpctr = regs.dcpctr().read();
    dcpctr.set_pid(pid);
    dcpctr.set_ccpl(ccpl);
    regs.dcpctr().write_value(dcpctr);
}

pub(super) fn set_pipe_nak_wait(pipe: u8) {
    // Match Renesas usb_cstd_set_nak(): clear BUF first, then wait for PBUSY to drop.
    if !(1..=9).contains(&pipe) {
        return;
    }
    set_pipe_pid(pipe, PipePid::Nak);

    for _ in 0..CFIFO_READY_SPINS {
        if !pipe_busy(pipe) {
            return;
        }
    }
}

pub(super) fn configure_pipe(
    regs: pac::usbfs::Usbfs,
    brdyenb_shadow: &mut u16,
    nrdyenb_shadow: &mut u16,
    bempenb_shadow: &mut u16,
    pipe: u8,
    binding: PipeBinding,
) {
    let pipe_type = match binding.ep_type {
        EndpointType::Bulk => Type::_01,
        EndpointType::Interrupt => Type::_10,
        _ => return,
    };

    clear_pipe_config(regs, pipe);
    regs.pipesel().write_value(Pipesel(pipe as u16));
    let mut pipecfg = Pipecfg::default();
    pipecfg.set_epnum(binding.ep_addr.index() as u8);
    pipecfg.set_dir(binding.ep_addr.direction() == UsbDirection::In);
    pipecfg.set_type_(pipe_type);
    pipecfg.set_dblb(matches!(binding.ep_type, EndpointType::Bulk));
    pipecfg.set_bfre(binding.ep_addr.direction() == UsbDirection::In);
    pipecfg.set_shtnak(
        binding.ep_addr.direction() == UsbDirection::Out
            && matches!(binding.ep_type, EndpointType::Bulk),
    );
    regs.pipecfg().write_value(pipecfg);
    regs.pipemaxp()
        .write_value(Pipemaxp(binding.max_packet & 0x01ff));
    regs.pipeperi().write_value(Pipeperi(0));
    regs.pipesel().write_value(Pipesel(0));
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

pub(super) fn clear_pipe_config(regs: pac::usbfs::Usbfs, pipe: u8) {
    set_pipe_nak_wait(pipe);
    regs.pipesel().write_value(Pipesel(pipe as u16));
    regs.pipecfg().write_value(Pipecfg(0));
    regs.pipemaxp().write_value(Pipemaxp(0));
    regs.pipeperi().write_value(Pipeperi(0));
    regs.pipesel().write_value(Pipesel(0));
    reset_pipe_control(pipe);
    clear_brdy(regs, pipe);
    clear_nrdy(regs, pipe);
    clear_bemp(regs, pipe);
}

pub(super) fn start_out_receive(
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
    set_pipe_pid(pipe, PipePid::Buf);
    write_brdyenb(regs, brdyenb_shadow, *brdyenb_shadow | bit);
    write_nrdyenb(regs, nrdyenb_shadow, *nrdyenb_shadow | bit);
}
