//! usb-device integration layer.

use core::{cell::RefCell, marker::PhantomData};

use usb_device::{
    Result as UsbResult, UsbDirection, UsbError,
    bus::{PollResult, UsbBus},
    endpoint::{EndpointAddress, EndpointType},
};

use super::types::{BusState, PipeBinding, UsbEventSnapshot};
use super::{Driver, Instance, UsbIrqEvent, regs};

/// `usb-device` bus adapter over the HAL USB driver.
pub struct Bus<'d, I: Instance> {
    _phantom: PhantomData<&'d I>,
    driver: Driver<'d, I>,
    state: RefCell<BusState>,
    events: RefCell<UsbEventSnapshot>,
}

impl<'d, I: Instance> Bus<'d, I> {
    /// Creates a new bus adapter around the HAL USB driver.
    pub fn new(driver: Driver<'d, I>) -> Self {
        Self {
            _phantom: PhantomData,
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
        if (regs.intsts0().read().0 & regs::USB_VALID) != 0 {
            regs::clear_intsts0(regs, regs::USB_VALID);
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
        if ep_type != EndpointType::Control || max_packet_size > 64 {
            if max_packet_size > 64 {
                return Err(UsbError::Unsupported);
            }
            return self.alloc_non_control_ep(ep_dir, ep_addr, ep_type, max_packet_size);
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

        let preserved_syscfg = regs.syscfg().read().0 & (0x0400 | 0x0001 | regs::USB_DPRPU);
        regs.dvstctr0().write_value(Default::default());
        regs.dcpctr()
            .write_value(crate::pac::usbfs::regs::Dcpctr(regs::USB_SQSET));
        regs.brdyenb().write_value(Default::default());
        regs.nrdyenb().write_value(Default::default());
        regs.bempenb().write_value(Default::default());
        regs.brdysts().write_value(Default::default());
        regs.nrdysts().write_value(Default::default());
        regs.bempsts().write_value(Default::default());
        regs.syscfg()
            .write_value(crate::pac::usbfs::regs::Syscfg(preserved_syscfg));
        regs.dcpcfg().write_value(Default::default());
        regs.dcpmaxp()
            .write_value(crate::pac::usbfs::regs::Dcpmaxp(self.ep0_size()));
        regs.cfifosel()
            .write_value(crate::pac::usbfs::regs::Cfifosel(regs::USB_MBW_8));
        regs.d0fifosel()
            .write_value(crate::pac::usbfs::regs::D0fifosel(regs::USB_MBW_16));
        regs.d1fifosel()
            .write_value(crate::pac::usbfs::regs::D1fifosel(regs::USB_MBW_8));

        let pipe_bindings = {
            let mut state = self.state.borrow_mut();
            state.ep0_stalled = false;
            state.ep0_short_in_waiting_status = false;
            state.ep0_setup_pending = false;
            state.ep0_setup_ready = false;
            state.ep0_expect_status_out = false;
            state.suspended = false;
            state.in_busy_mask = 0;
            state.brdyenb_shadow = 0;
            state.nrdyenb_shadow = 0;
            state.bempenb_shadow = 0;
            let bindings = state.pipe_bindings;
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, regs::USB_BRDY0);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, regs::USB_NRDY0, 0);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, regs::USB_BEMP0);
            *self.events.borrow_mut() = UsbEventSnapshot::default();
            bindings
        };

        regs.dcpctr().write_value(crate::pac::usbfs::regs::Dcpctr(
            regs::USB_SQCLR | regs::USB_PID_NAK,
        ));

        for pipe in 1..=9u8 {
            regs::clear_pipe_config(regs, pipe);
        }

        let mut state = self.state.borrow_mut();
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
            let current = regs::read_pipectr(pipe);
            regs::write_pipectr_reason(
                pipe,
                (current & !regs::USB_PID_MASK) | regs::USB_PID_NAK,
                16,
            );
            regs::clear_bemp(regs, pipe);
            regs.d0fifoctr()
                .write_value(crate::pac::usbfs::regs::D0fifoctr(regs::USB_BCLR));
            regs::write_d0fifo(regs, &buf[..write_len]);

            if write_len == 0 || write_len < binding.max_packet as usize {
                regs.d0fifoctr()
                    .write_value(crate::pac::usbfs::regs::D0fifoctr(regs::USB_BVAL));
            }

            regs::update_bempenb(regs, &mut state.bempenb_shadow, 1u16 << pipe, 0);
            let current = regs::read_pipectr(pipe);
            regs::write_pipectr_reason(
                pipe,
                (current & !regs::USB_PID_MASK) | regs::USB_PID_BUF,
                17,
            );
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
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, regs::USB_BRDY0);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, 0, regs::USB_NRDY0);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, regs::USB_BEMP0);
            let dcpctr = regs.dcpctr().read().0;
            regs.dcpctr().write_value(crate::pac::usbfs::regs::Dcpctr(
                (dcpctr & !regs::USB_PID_MASK) | regs::USB_CCPL | regs::USB_PID_BUF,
            ));
            state.ep0_last_setup_dir_out = false;
            return Ok(0);
        }

        drop(state);

        if !regs::select_pipe0_ready(regs, true) {
            return Err(UsbError::WouldBlock);
        }

        regs.cfifoctr()
            .write_value(crate::pac::usbfs::regs::Cfifoctr(regs::USB_BCLR));

        let ep0_max_packet = self.ep0_size() as usize;
        let mut state = self.state.borrow_mut();
        let dcpctr = regs.dcpctr().read().0;
        regs.dcpctr().write_value(crate::pac::usbfs::regs::Dcpctr(
            (dcpctr & !regs::USB_PID_MASK) | regs::USB_PID_NAK,
        ));
        regs::clear_bemp0(regs);
        regs::write_cfifo(regs, buf);

        let short_packet = buf.len() < ep0_max_packet;
        if buf.is_empty() || short_packet {
            regs.cfifoctr()
                .write_value(crate::pac::usbfs::regs::Cfifoctr(regs::USB_BVAL));
        }

        if short_packet {
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, regs::USB_BEMP0);
        } else {
            regs::update_bempenb(regs, &mut state.bempenb_shadow, regs::USB_BEMP0, 0);
        }

        let current = regs.dcpctr().read().0;
        regs.dcpctr().write_value(crate::pac::usbfs::regs::Dcpctr(
            (current & !regs::USB_PID_MASK) | regs::USB_PID_BUF,
        ));
        state.ep0_short_in_waiting_status = short_packet;
        Ok(buf.len())
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

            let fifoctr = regs.cfifoctr().read().0;
            let dtln = (fifoctr & regs::USB_DTLN) as usize;
            if dtln == 0 {
                regs.cfifoctr()
                    .write_value(crate::pac::usbfs::regs::Cfifoctr(regs::USB_BCLR));
                regs::clear_brdy(regs, pipe);
                let mut state = self.state.borrow_mut();
                self.start_out_receive_in_state(
                    &mut state,
                    pipe,
                    binding.max_packet,
                    binding.max_packet as u32,
                );
                return Err(UsbError::WouldBlock);
            }
            if dtln > buf.len() {
                return Err(UsbError::BufferOverflow);
            }

            regs::read_cfifo(regs, &mut buf[..dtln]);
            regs.cfifoctr()
                .write_value(crate::pac::usbfs::regs::Cfifoctr(regs::USB_BCLR));
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
            regs::update_brdyenb(regs, &mut state.brdyenb_shadow, 0, regs::USB_BRDY0);
            regs::update_nrdyenb(regs, &mut state.nrdyenb_shadow, 0, regs::USB_NRDY0);
            regs::update_bempenb(regs, &mut state.bempenb_shadow, 0, regs::USB_BEMP0);
            let dcpctr = regs.dcpctr().read().0;
            regs.dcpctr().write_value(crate::pac::usbfs::regs::Dcpctr(
                (dcpctr & !regs::USB_PID_MASK) | regs::USB_CCPL | regs::USB_PID_BUF,
            ));
            return Ok(0);
        }

        if !regs::select_pipe0_ready(regs, false) {
            return Err(UsbError::WouldBlock);
        }

        let fifoctr = regs.cfifoctr().read().0;
        let dtln = (fifoctr & regs::USB_DTLN) as usize;
        if dtln > buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        if dtln == 0 {
            regs.cfifoctr()
                .write_value(crate::pac::usbfs::regs::Cfifoctr(regs::USB_BCLR));
            regs::clear_brdy0(regs);
            return Ok(0);
        }

        regs::read_cfifo(regs, &mut buf[..dtln]);
        regs::clear_brdy0(regs);
        Ok(dtln)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() != 0 {
            return;
        }

        let regs = self.regs();
        let current = regs.dcpctr().read().0;
        let live_ctsq = regs.intsts0().read().0 & regs::USB_CTSQ;
        let next_pid = if stalled {
            regs::USB_PID_STALL
        } else if live_ctsq == regs::USB_CS_RDDS || live_ctsq == regs::USB_CS_RDSS {
            regs::USB_PID_BUF
        } else {
            regs::USB_PID_NAK
        };
        let next = (current & !regs::USB_PID_MASK) | next_pid;
        let extra = if stalled || next_pid == regs::USB_PID_BUF {
            0
        } else {
            regs::USB_SQCLR
        };
        regs.dcpctr()
            .write_value(crate::pac::usbfs::regs::Dcpctr(next | extra));

        let mut state = self.state.borrow_mut();
        if !stalled && next_pid == regs::USB_PID_BUF && live_ctsq == regs::USB_CS_RDDS {
            state.ep0_expect_status_out = true;
        }
        state.ep0_stalled = stalled;
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        ep_addr.index() == 0 && self.state.borrow().ep0_stalled
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
        let intsts0 = latched.intsts0.0;

        if (intsts0 & regs::USB_DVST) != 0 {
            regs::clear_intsts0(regs, regs::USB_DVST);
            if (intsts0 & regs::USB_DVSQ) == regs::USB_DS_DFLT {
                return PollResult::Reset;
            }
        }

        if (intsts0 & regs::USB_SOFR) != 0 {
            regs::clear_intsts0(regs, regs::USB_SOFR);
        }

        if (intsts0 & regs::USB_RESM) != 0 {
            regs::clear_intsts0(regs, regs::USB_RESM);
            return PollResult::Resume;
        }

        if latched.nrdysts.0 != 0 {
            if (latched.nrdysts.0 & regs::USB_NRDY0) != 0 {
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

                let current = regs::read_pipectr(pipe);
                if (current & regs::USB_PID_MASK) != regs::USB_PID_BUF
                    && (current & regs::USB_PBUSY) == 0
                {
                    regs::write_pipectr_reason(
                        pipe,
                        (current & !regs::USB_PID_MASK) | regs::USB_PID_BUF,
                        19,
                    );
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

                let current = regs::read_pipectr(pipe);
                let pid = current & regs::USB_PID_MASK;
                let busy = (current & regs::USB_PBUSY) != 0;
                if pid == regs::USB_PID_BUF || busy {
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

        let non_ep_bemp = latched.bempsts.0 & !regs::USB_BEMP0;
        if non_ep_bemp != 0 {
            let mut state = self.state.borrow_mut();
            for pipe in 1..=9u8 {
                let bit = 1u16 << pipe;
                if (non_ep_bemp & bit) == 0 {
                    continue;
                }
                regs::clear_bemp(regs, pipe);
                let next_bemp = state.bempenb_shadow & !bit;
                self.write_bempenb_in_state(&mut state, next_bemp);
                let current = regs::read_pipectr(pipe);
                regs::write_pipectr_reason(
                    pipe,
                    (current & !regs::USB_PID_MASK) | regs::USB_PID_NAK,
                    18,
                );
                state.in_busy_mask &= !bit;
            }
        }

        let ctrt_ctsq = latched.ctrt_ctsq;
        if (intsts0 & regs::USB_CTRT) != 0 {
            regs::clear_intsts0(regs, regs::USB_CTRT);
        }

        let queued_setup_ready = latched.setup_valid;
        let control_setup = !queued_setup_ready
            && matches!(
                ctrt_ctsq,
                regs::USB_CS_RDDS | regs::USB_CS_WRDS | regs::USB_CS_WRND
            );
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
        if (brdysts & regs::USB_BRDY0) != 0 && ep_setup == 0 {
            ep_out |= 1;
        } else if ctrt_ctsq == regs::USB_CS_RDSS {
            let mut state = self.state.borrow_mut();
            state.ep0_expect_status_out = false;
            state.ep0_short_in_waiting_status = false;
            ep_out |= 1;
        }

        if (brdysts & !regs::USB_BRDY0) != 0 {
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

        let ep_in_complete = if (latched.bempsts.0 & regs::USB_BEMP0) != 0 {
            regs::clear_bemp0(regs);
            if self.state.borrow().ep0_short_in_waiting_status {
                0
            } else {
                1
            }
        } else {
            0
        };

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

unsafe impl<'d, I: Instance> Sync for Bus<'d, I> {}
