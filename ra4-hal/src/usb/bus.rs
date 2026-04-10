//! usb-device integration layer.

use core::cell::RefCell;

use usb_device::{
    Result as UsbResult, UsbDirection, UsbError,
    bus::{PollResult, UsbBus},
    endpoint::{EndpointAddress, EndpointType},
};

use crate::pac::usbfs::regs::{Cfifosel, D0fifosel, D1fifosel, Dcpctr};
use crate::pac::usbfs::vals::{Ctsq, DcpctrPid, Dvsq};

use super::types::{BusState, PipeBinding, UsbEventSnapshot};
use super::{Driver, Instance, UsbIrqEvent, regs};

/// `usb-device` bus adapter over the HAL USB driver.
pub struct Bus<'d, I: Instance> {
    driver: Driver<'d, I>,
    state: RefCell<BusState>,
    events: RefCell<UsbEventSnapshot>,
}

impl<'d, I: Instance> Bus<'d, I> {
    /// Creates a new bus adapter around the HAL USB driver.
    pub fn new(driver: Driver<'d, I>) -> Self {
        Self {
            driver,
            state: RefCell::new(BusState::default()),
            events: RefCell::new(UsbEventSnapshot::default()),
        }
    }

    /// Returns a shared reference to the underlying driver.
    pub fn driver(&self) -> &Driver<'d, I> {
        &self.driver
    }

    /// Returns a mutable reference to the underlying driver.
    pub fn driver_mut(&mut self) -> &mut Driver<'d, I> {
        &mut self.driver
    }

    /// Merges one captured interrupt event into the bus-side snapshot.
    pub fn merge_irq_event(&self, event: &UsbIrqEvent) {
        self.events.borrow_mut().merge_irq_event(event);
    }

    /// Takes the current merged event snapshot.
    pub fn take_event_snapshot(&self) -> UsbEventSnapshot {
        core::mem::take(&mut *self.events.borrow_mut())
    }

    /// Returns shared access to the bus state used by endpoint and control logic.
    pub fn state(&self) -> &RefCell<BusState> {
        &self.state
    }

    fn regs(&self) -> crate::pac::usbfs::Usbfs {
        regs::usbfs()
    }

    fn ep0_size(&self) -> u16 {
        self.state.borrow().ep0_max_packet
    }

    fn stage_setup_packet(&self, setup_packet: [u8; 8]) {
        let regs = self.regs();
        if regs.intsts0().read().valid() {
            regs::clear_valid(regs);
        }

        let mut state = self.state.borrow_mut();
        state.ep0_setup_packet = setup_packet;
        state.ep0_setup_pending = true;
        state.ep0_setup_ready = true;
    }

    fn find_pipe(&self, ep_addr: EndpointAddress) -> Option<(u8, PipeBinding)> {
        let state = self.state.borrow();
        state
            .pipe_bindings
            .iter()
            .enumerate()
            .skip(1)
            .find_map(|(pipe, binding)| match binding {
                Some(binding) if binding.ep_addr == ep_addr => Some((pipe as u8, *binding)),
                _ => None,
            })
    }

    fn configure_pipe_in_state(&self, state: &mut BusState, pipe: u8, binding: PipeBinding) {
        let regs = self.regs();
        let mut brdy = state.brdyenb_shadow;
        let mut nrdy = state.nrdyenb_shadow;
        let mut bemp = state.bempenb_shadow;
        regs::configure_pipe(regs, &mut brdy, &mut nrdy, &mut bemp, pipe, binding);
        state.brdyenb_shadow = brdy;
        state.nrdyenb_shadow = nrdy;
        state.bempenb_shadow = bemp;
    }

    fn start_out_receive_in_state(
        &self,
        state: &mut BusState,
        pipe: u8,
        max_packet: u16,
        requested_len: u32,
    ) {
        let regs = self.regs();
        let mut brdy = state.brdyenb_shadow;
        let mut nrdy = state.nrdyenb_shadow;
        regs::start_out_receive(regs, &mut brdy, &mut nrdy, pipe, max_packet, requested_len);
        state.brdyenb_shadow = brdy;
        state.nrdyenb_shadow = nrdy;
    }

    fn write_bempenb_in_state(&self, state: &mut BusState, mask: u16) {
        let regs = self.regs();
        let mut bemp = state.bempenb_shadow;
        regs::write_bempenb(regs, &mut bemp, mask);
        state.bempenb_shadow = bemp;
    }

    fn alloc_non_control_ep(
        &self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
    ) -> UsbResult<EndpointAddress> {
        match ep_type {
            EndpointType::Bulk | EndpointType::Interrupt => {}
            _ => return Err(UsbError::Unsupported),
        }

        let mut state = self.state.borrow_mut();
        let requested = ep_addr.filter(|addr| addr.index() != 0);
        let requested_index = requested.map(|addr| addr.index());
        let current_mask = match ep_dir {
            UsbDirection::In => state.allocated_in_mask,
            UsbDirection::Out => state.allocated_out_mask,
        };

        let index = if let Some(addr) = requested {
            if addr.direction() != ep_dir {
                return Err(UsbError::InvalidEndpoint);
            }

            let bit = 1u16 << addr.index();
            if (current_mask & bit) != 0 {
                return Err(UsbError::InvalidEndpoint);
            }

            addr.index()
        } else {
            let mut free_index = None;
            for idx in 1..16 {
                let bit = 1u16 << idx;
                if (current_mask & bit) == 0 {
                    free_index = Some(idx);
                    break;
                }
            }
            free_index.ok_or(UsbError::EndpointOverflow)?
        };

        if index == 0 || requested_index == Some(0) {
            return Err(UsbError::InvalidEndpoint);
        }

        let pipe = match ep_type {
            EndpointType::Bulk => {
                (1..=5).find(|pipe| state.pipe_bindings[*pipe as usize].is_none())
            }
            EndpointType::Interrupt => {
                (6..=9).find(|pipe| state.pipe_bindings[*pipe as usize].is_none())
            }
            _ => None,
        }
        .ok_or(UsbError::EndpointOverflow)?;

        match ep_dir {
            UsbDirection::In => state.allocated_in_mask |= 1u16 << index,
            UsbDirection::Out => state.allocated_out_mask |= 1u16 << index,
        }

        let ep_addr = EndpointAddress::from_parts(index, ep_dir);
        let binding = PipeBinding {
            ep_addr,
            ep_type,
            max_packet: max_packet_for_pipe(ep_type, pipe, max_packet_size)?,
        };
        state.pipe_bindings[pipe as usize] = Some(binding);

        self.configure_pipe_in_state(&mut state, pipe, binding);
        if ep_dir == UsbDirection::Out {
            self.start_out_receive_in_state(
                &mut state,
                pipe,
                binding.max_packet,
                binding.max_packet as u32,
            );
        }

        Ok(ep_addr)
    }
}

impl<'d, I: Instance + 'static> UsbBus for Bus<'d, I> {
    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> UsbResult<EndpointAddress> {
        if ep_type != EndpointType::Control {
            return self.alloc_non_control_ep(ep_dir, ep_addr, ep_type, max_packet_size);
        }

        if max_packet_size > 64 {
            return Err(UsbError::Unsupported);
        }

        let requested = ep_addr.unwrap_or_else(|| match ep_dir {
            UsbDirection::Out => EndpointAddress::from_parts(0, UsbDirection::Out),
            UsbDirection::In => EndpointAddress::from_parts(0, UsbDirection::In),
        });

        if requested.index() != 0 || requested.direction() != ep_dir {
            return Err(UsbError::InvalidEndpoint);
        }

        let mut state = self.state.borrow_mut();
        if state.ep0_max_packet == 0 {
            state.ep0_max_packet = max_packet_size;
        } else if state.ep0_max_packet != max_packet_size {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        match ep_dir {
            UsbDirection::Out if state.ep0_out_allocated => Err(UsbError::InvalidEndpoint),
            UsbDirection::In if state.ep0_in_allocated => Err(UsbError::InvalidEndpoint),
            UsbDirection::Out => {
                state.ep0_out_allocated = true;
                Ok(requested)
            }
            UsbDirection::In => {
                state.ep0_in_allocated = true;
                Ok(requested)
            }
        }
    }

    fn enable(&mut self) {
        self.reset();
        self.driver.attach();
    }

    fn reset(&self) {
        let regs = self.regs();
        Driver::<'d, I>::clear_capture_state_static();

        let live_syscfg = regs.syscfg().read();
        let mut preserved_syscfg = crate::pac::usbfs::regs::Syscfg::default();
        preserved_syscfg.set_usbe(live_syscfg.usbe());
        preserved_syscfg.set_dprpu(live_syscfg.dprpu());
        preserved_syscfg.set_scke(live_syscfg.scke());
        regs.dvstctr0().write_value(Default::default());
        let mut dcpctr = Dcpctr::default();
        dcpctr.set_sqset(true);
        regs.dcpctr().write_value(dcpctr);
        regs.brdyenb().write_value(Default::default());
        regs.nrdyenb().write_value(Default::default());
        regs.bempenb().write_value(Default::default());
        regs.brdysts().write_value(Default::default());
        regs.nrdysts().write_value(Default::default());
        regs.bempsts().write_value(Default::default());
        regs.syscfg().write_value(preserved_syscfg);
        regs.dcpcfg().write_value(Default::default());
        regs.dcpmaxp()
            .write_value(crate::pac::usbfs::regs::Dcpmaxp(self.ep0_size()));
        let mut cfifosel = Cfifosel::default();
        cfifosel.set_mbw(false);
        regs.cfifosel().write_value(cfifosel);
        let mut d0fifosel = D0fifosel::default();
        d0fifosel.set_mbw(true);
        regs.d0fifosel().write_value(d0fifosel);
        let mut d1fifosel = D1fifosel::default();
        d1fifosel.set_mbw(false);
        regs.d1fifosel().write_value(d1fifosel);

        let pipe_bindings = {
            let mut state = self.state.borrow_mut();
            state.ep0_stalled = false;
            state.stalled_in_mask = 0;
            state.stalled_out_mask = 0;
            state.ep0_short_in_waiting_status = false;
            state.ep0_setup_pending = false;
            state.ep0_setup_ready = false;
            state.ep0_last_setup_dir_out = false;
            state.ep0_expect_status_out = false;
            state.suspended = false;
            state.in_busy_mask = 0;
            state.brdyenb_shadow = 0;
            state.nrdyenb_shadow = 0;
            state.bempenb_shadow = 0;
            let bindings = state.pipe_bindings;
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, 1u16);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, 1u16, 0);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, 1u16);
            *self.events.borrow_mut() = UsbEventSnapshot::default();
            bindings
        };

        let mut dcpctr = Dcpctr::default();
        dcpctr.set_sqclr(true);
        dcpctr.set_pid(DcpctrPid::_00);
        regs.dcpctr().write_value(dcpctr);

        let mut state = self.state.borrow_mut();
        let mut brdyenb_shadow = state.brdyenb_shadow;
        let mut nrdyenb_shadow = state.nrdyenb_shadow;
        let mut bempenb_shadow = state.bempenb_shadow;
        for pipe in 1..=9u8 {
            regs::clear_pipe_config(
                regs,
                &mut brdyenb_shadow,
                &mut nrdyenb_shadow,
                &mut bempenb_shadow,
                pipe,
            );
        }
        state.brdyenb_shadow = brdyenb_shadow;
        state.nrdyenb_shadow = nrdyenb_shadow;
        state.bempenb_shadow = bempenb_shadow;
        for (pipe, binding) in pipe_bindings.iter().enumerate().skip(1) {
            if let Some(binding) = binding {
                self.configure_pipe_in_state(&mut state, pipe as u8, *binding);
                if binding.ep_addr.direction() == UsbDirection::Out {
                    self.start_out_receive_in_state(
                        &mut state,
                        pipe as u8,
                        binding.max_packet,
                        binding.max_packet as u32,
                    );
                }
            }
        }
    }

    fn set_device_address(&self, _addr: u8) {}

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        let regs = self.regs();

        if ep_addr.index() != 0 {
            if ep_addr.direction() != UsbDirection::In {
                return Err(UsbError::InvalidEndpoint);
            }

            let Some((pipe, binding)) = self.find_pipe(ep_addr) else {
                return Err(UsbError::InvalidEndpoint);
            };

            if (self.state.borrow().in_busy_mask & (1u16 << pipe)) != 0 {
                return Err(UsbError::WouldBlock);
            }

            let write_len = buf.len().min(binding.max_packet as usize);
            regs::select_d0fifo(regs, pipe);
            if !regs::d0fifo_ready(regs) {
                return Err(UsbError::WouldBlock);
            }

            let mut state = self.state.borrow_mut();
            regs::set_pipe_pid(pipe, regs::PipePid::Nak);
            regs::clear_bemp(regs, pipe);
            regs::clear_d0fifo_buffer(regs);
            regs::write_d0fifo(regs, &buf[..write_len]);

            if write_len == 0 || write_len < binding.max_packet as usize {
                regs::set_d0fifo_bval(regs);
            }

            regs::update_bempenb(regs, &mut state.bempenb_shadow, 1u16 << pipe, 0);
            regs::set_pipe_pid(pipe, regs::PipePid::Buf);
            state.in_busy_mask |= 1u16 << pipe;
            return Ok(write_len);
        }

        if ep_addr.direction() != UsbDirection::In {
            return Err(UsbError::InvalidEndpoint);
        }

        let mut state = self.state.borrow_mut();
        if state.ep0_stalled {
            return Err(UsbError::WouldBlock);
        }

        if state.ep0_last_setup_dir_out && buf.is_empty() {
            regs::clear_bemp0(regs);
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, 1u16);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, 0, 1u16);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, 1u16);
            regs::set_dcp_pid(regs, DcpctrPid::_01, true);
            state.ep0_last_setup_dir_out = false;
            return Ok(0);
        }

        drop(state);

        if !regs::select_pipe0_ready(regs, true) {
            return Err(UsbError::WouldBlock);
        }

        regs::clear_cfifo_buffer(regs);

        let ep0_max_packet = self.ep0_size() as usize;
        let write_len = buf.len().min(ep0_max_packet);
        let mut state = self.state.borrow_mut();
        regs::set_dcp_pid(regs, DcpctrPid::_00, false);
        regs::clear_bemp0(regs);
        regs::write_cfifo(regs, &buf[..write_len]);

        let short_packet = write_len < ep0_max_packet;
        if write_len == 0 || short_packet {
            regs::set_cfifo_bval(regs);
        }

        if short_packet {
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, 1u16);
        } else {
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 1u16, 0);
        }

        regs::set_dcp_pid(regs, DcpctrPid::_01, false);
        state.ep0_short_in_waiting_status = short_packet;
        Ok(write_len)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        let regs = self.regs();

        if ep_addr.index() != 0 {
            if ep_addr.direction() != UsbDirection::Out {
                return Err(UsbError::InvalidEndpoint);
            }

            let Some((pipe, binding)) = self.find_pipe(ep_addr) else {
                return Err(UsbError::InvalidEndpoint);
            };

            regs::select_cfifo(regs, pipe, false, false);
            if !regs::cfifo_ready(regs) {
                return Err(UsbError::WouldBlock);
            }

            let dtln = regs::cfifo_dtln(regs);
            if dtln == 0 {
                regs::clear_cfifo_buffer(regs);
                regs::clear_brdy(regs, pipe);
                let mut state = self.state.borrow_mut();
                self.start_out_receive_in_state(
                    &mut state,
                    pipe,
                    binding.max_packet,
                    binding.max_packet as u32,
                );
                return Ok(0);
            }
            if dtln > buf.len() {
                return Err(UsbError::BufferOverflow);
            }

            regs::read_cfifo(regs, &mut buf[..dtln]);
            regs::clear_cfifo_buffer(regs);
            regs::clear_brdy(regs, pipe);
            let mut state = self.state.borrow_mut();
            self.start_out_receive_in_state(
                &mut state,
                pipe,
                binding.max_packet,
                binding.max_packet as u32,
            );
            return Ok(dtln);
        }

        if ep_addr.direction() != UsbDirection::Out {
            return Err(UsbError::InvalidEndpoint);
        }

        if self.state.borrow().ep0_setup_ready {
            if buf.len() < 8 {
                return Err(UsbError::BufferOverflow);
            }

            let mut state = self.state.borrow_mut();
            buf[..8].copy_from_slice(&state.ep0_setup_packet);
            state.ep0_last_setup_dir_out = (state.ep0_setup_packet[0] & 0x80) == 0;
            state.ep0_setup_pending = false;
            state.ep0_setup_ready = false;
            return Ok(8);
        }

        if buf.is_empty() {
            let mut state = self.state.borrow_mut();
            regs::clear_bemp0(regs);
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, 1u16);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, 0, 1u16);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, 1u16);
            regs::set_dcp_pid(regs, DcpctrPid::_01, true);
            return Ok(0);
        }

        if !regs::select_pipe0_ready(regs, false) {
            return Err(UsbError::WouldBlock);
        }

        let dtln = regs::cfifo_dtln(regs);
        if dtln > buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        if dtln == 0 {
            regs::clear_cfifo_buffer(regs);
            regs::clear_brdy0(regs);
            return Ok(0);
        }

        regs::read_cfifo(regs, &mut buf[..dtln]);
        regs::clear_brdy0(regs);
        Ok(dtln)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() == 0 {
            let regs = self.regs();
            let live_ctsq = regs.intsts0().read().ctsq();
            let next_pid = if stalled {
                DcpctrPid::_10
            } else if live_ctsq == Ctsq::_001 || live_ctsq == Ctsq::_010 {
                DcpctrPid::_01
            } else {
                DcpctrPid::_00
            };
            let mut dcpctr = regs.dcpctr().read();
            dcpctr.set_pid(next_pid);
            dcpctr.set_sqclr(!stalled && next_pid != DcpctrPid::_01);
            regs.dcpctr().write_value(dcpctr);

            let mut state = self.state.borrow_mut();
            if !stalled && next_pid == DcpctrPid::_01 && live_ctsq == Ctsq::_001 {
                state.ep0_expect_status_out = true;
            }
            state.ep0_stalled = stalled;
            return;
        }

        let Some((pipe, binding)) = self.find_pipe(ep_addr) else {
            return;
        };

        let regs = self.regs();
        let bit = 1u16 << ep_addr.index();
        let mut state = self.state.borrow_mut();

        if stalled {
            regs::set_pipe_pid(pipe, regs::PipePid::Stall);
            match ep_addr.direction() {
                UsbDirection::In => {
                    state.stalled_in_mask |= bit;
                    state.in_busy_mask &= !bit;
                }
                UsbDirection::Out => state.stalled_out_mask |= bit,
            }
            return;
        }

        regs::reset_pipe_control(pipe);
        match ep_addr.direction() {
            UsbDirection::In => {
                state.stalled_in_mask &= !bit;
                state.in_busy_mask &= !bit;
            }
            UsbDirection::Out => {
                state.stalled_out_mask &= !bit;
                self.start_out_receive_in_state(
                    &mut state,
                    pipe,
                    binding.max_packet,
                    binding.max_packet as u32,
                );
            }
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() == 0 {
            return self.state.borrow().ep0_stalled;
        }

        let bit = 1u16 << ep_addr.index();
        let state = self.state.borrow();
        match ep_addr.direction() {
            UsbDirection::In => (state.stalled_in_mask & bit) != 0,
            UsbDirection::Out => (state.stalled_out_mask & bit) != 0,
        }
    }

    fn suspend(&self) {
        self.state.borrow_mut().suspended = true;
    }

    fn resume(&self) {
        self.state.borrow_mut().suspended = false;
    }

    fn poll(&self) -> PollResult {
        while self.driver.irq_pending() {
            let event = self.driver.take_events();
            self.merge_irq_event(&event);
        }

        let regs = self.regs();
        let latched = self.take_event_snapshot();

        if latched.intsts0.dvst() {
            regs::clear_dvst(regs);
            if latched.intsts0.dvsq() == Dvsq::_001 {
                return PollResult::Reset;
            }
        }

        if latched.intsts0.sofr() {
            regs::clear_sofr(regs);
        }

        if latched.intsts0.resm() {
            regs::clear_resm(regs);
            return PollResult::Resume;
        }

        if latched.nrdysts.0 != 0 {
            if latched.nrdysts.nrdy(0) {
                regs::clear_nrdy0(regs);
            }

            let state = self.state.borrow_mut();
            for pipe in 1..=9u8 {
                let bit = 1u16 << pipe;
                if (latched.nrdysts.0 & bit) == 0 {
                    continue;
                }
                regs::clear_nrdy(regs, pipe);
                let Some(binding) = state.pipe_bindings[pipe as usize] else {
                    continue;
                };
                if binding.ep_addr.direction() != UsbDirection::Out {
                    continue;
                }

                if !regs::pipe_pid_is_buf(pipe) && !regs::pipe_busy(pipe) {
                    regs::set_pipe_pid(pipe, regs::PipePid::Buf);
                }
            }
        }

        {
            let mut state = self.state.borrow_mut();
            for pipe in 1..=9u8 {
                let Some(binding) = state.pipe_bindings[pipe as usize] else {
                    continue;
                };
                if binding.ep_addr.direction() != UsbDirection::Out {
                    continue;
                }

                if regs::pipe_pid_is_buf(pipe) || regs::pipe_busy(pipe) {
                    continue;
                }

                self.start_out_receive_in_state(
                    &mut state,
                    pipe,
                    binding.max_packet,
                    binding.max_packet as u32,
                );
            }
        }

        let mut ep_in_complete = 0u16;
        let non_ep_bemp = latched.bempsts.0 & !1u16;
        if non_ep_bemp != 0 {
            let mut state = self.state.borrow_mut();
            for pipe in 1..=9u8 {
                let bit = 1u16 << pipe;
                if (non_ep_bemp & bit) == 0 {
                    continue;
                }
                let Some(binding) = state.pipe_bindings[pipe as usize] else {
                    continue;
                };
                regs::clear_bemp(regs, pipe);
                let next_bemp = state.bempenb_shadow & !bit;
                self.write_bempenb_in_state(&mut state, next_bemp);
                regs::set_pipe_pid(pipe, regs::PipePid::Nak);
                state.in_busy_mask &= !bit;
                if binding.ep_addr.direction() == UsbDirection::In {
                    ep_in_complete |= 1u16 << binding.ep_addr.index();
                }
            }
        }

        let ctrt_ctsq = Ctsq::from_bits(latched.ctrt_ctsq as u8);
        if latched.intsts0.ctrt() {
            regs::clear_ctrt(regs);
        }

        let queued_setup_ready = latched.setup_valid;
        let control_setup =
            !queued_setup_ready && matches!(ctrt_ctsq, Ctsq::_001 | Ctsq::_011 | Ctsq::_101);
        let valid_setup = latched.valid_pending;

        let ep_setup = if control_setup || valid_setup || queued_setup_ready {
            if !self.state.borrow().ep0_setup_ready && queued_setup_ready {
                self.stage_setup_packet(latched.setup_packet);
            }
            if self.state.borrow().ep0_setup_ready {
                1
            } else {
                0
            }
        } else if self.state.borrow().ep0_setup_ready {
            1
        } else {
            0
        };

        let brdysts = latched.brdysts.0;
        let mut ep_out = 0u16;
        if latched.brdysts.brdy(0) && ep_setup == 0 {
            ep_out |= 1;
        } else if ctrt_ctsq == Ctsq::_010 {
            let mut state = self.state.borrow_mut();
            state.ep0_expect_status_out = false;
            state.ep0_short_in_waiting_status = false;
            ep_out |= 1;
        }

        if (brdysts & !1u16) != 0 {
            let state = self.state.borrow();
            for pipe in 1..=9u8 {
                let bit = 1u16 << pipe;
                if (brdysts & bit) == 0 {
                    continue;
                }
                let Some(binding) = state.pipe_bindings[pipe as usize] else {
                    continue;
                };
                if binding.ep_addr.direction() == UsbDirection::Out {
                    ep_out |= 1u16 << binding.ep_addr.index();
                }
            }
        }

        let ep0_in_complete = if latched.bempsts.bemp(0) {
            regs::clear_bemp0(regs);
            1
        } else {
            0
        };
        ep_in_complete |= ep0_in_complete;

        if ep_setup != 0 || ep_out != 0 || ep_in_complete != 0 {
            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        } else if self.state.borrow().suspended {
            PollResult::Suspend
        } else {
            PollResult::None
        }
    }

    fn force_reset(&self) -> UsbResult<()> {
        self.driver.force_reset().map_err(|_| UsbError::Unsupported)
    }
}

// SAFETY: `usb-device` requires `UsbBus: Sync`, but this implementation only
// targets the single-core MCU execution model used by the HAL. Access to the
// `RefCell` state inside `Bus` is expected to be serialized by the USB stack's
// call pattern and by ISR/mainline coordination on that single core; it is not
// intended for true multi-threaded concurrent access across CPUs.
unsafe impl<'d, I: Instance> Sync for Bus<'d, I> {}

fn max_packet_for_pipe(ep_type: EndpointType, pipe: u8, requested: u16) -> UsbResult<u16> {
    match ep_type {
        EndpointType::Bulk if pipe <= 2 && requested <= 256 && requested != 0 => Ok(requested),
        EndpointType::Bulk if (3..=5).contains(&pipe) => match requested {
            8 | 16 | 32 | 64 => Ok(requested),
            _ => Err(UsbError::Unsupported),
        },
        EndpointType::Interrupt if (6..=9).contains(&pipe) && requested <= 64 && requested != 0 => {
            Ok(requested)
        }
        _ => Err(UsbError::Unsupported),
    }
}
