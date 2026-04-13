use crate::pac::usbfs::regs::{Bempsts, Brdysts, Intsts0, Intsts1, Nrdysts};
use usb_device::endpoint::{EndpointAddress, EndpointType};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Error {
    #[default]
    UnsupportedClock,
    EndpointOverflow,
    InvalidEndpoint,
    Busy,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsbIrqEvent {
    pub intsts0: Intsts0,
    pub intsts1: Intsts1,
    pub ctrt_ctsq: u16,
    pub seqno: u32,
    pub valid_high: bool,
    pub valid_rising: bool,
    pub brdysts: Brdysts,
    pub nrdysts: Nrdysts,
    pub bempsts: Bempsts,
    pub setup_valid: bool,
    pub setup_packet: [u8; 8],
}

impl UsbIrqEvent {
    pub const fn is_empty(self) -> bool {
        self.intsts0.0 == 0
            && self.intsts1.0 == 0
            && !self.valid_high
            && !self.valid_rising
            && self.brdysts.0 == 0
            && self.nrdysts.0 == 0
            && self.bempsts.0 == 0
            && !self.setup_valid
    }

    pub const fn merged(self, newer: Self) -> Self {
        Self {
            intsts0: Intsts0(self.intsts0.0 | newer.intsts0.0),
            intsts1: Intsts1(self.intsts1.0 | newer.intsts1.0),
            ctrt_ctsq: if newer.intsts0.ctrt() {
                newer.ctrt_ctsq
            } else {
                self.ctrt_ctsq
            },
            seqno: if newer.seqno != 0 {
                newer.seqno
            } else {
                self.seqno
            },
            valid_high: self.valid_high || newer.valid_high,
            valid_rising: self.valid_rising || newer.valid_rising,
            brdysts: Brdysts(self.brdysts.0 | newer.brdysts.0),
            nrdysts: Nrdysts(self.nrdysts.0 | newer.nrdysts.0),
            bempsts: Bempsts(self.bempsts.0 | newer.bempsts.0),
            setup_valid: self.setup_valid || newer.setup_valid,
            setup_packet: if newer.setup_valid {
                newer.setup_packet
            } else {
                self.setup_packet
            },
        }
    }

    pub fn merge_from(&mut self, newer: Self) {
        *self = self.merged(newer);
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsbIrqLocalState {
    pub capture_valid_latched: bool,
    pub capture_seqno: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct UsbEventSnapshot {
    pub intsts0: Intsts0,
    pub ctrt_ctsq: u16,
    pub brdysts: Brdysts,
    pub nrdysts: Nrdysts,
    pub bempsts: Bempsts,
    pub valid_pending: bool,
    pub setup_valid: bool,
    pub setup_packet: [u8; 8],
}

impl UsbEventSnapshot {
    pub fn merge_irq_event(&mut self, event: &UsbIrqEvent) {
        self.intsts0 = Intsts0(self.intsts0.0 | event.intsts0.0);
        if event.intsts0.ctrt() {
            self.ctrt_ctsq = event.ctrt_ctsq;
        }
        self.brdysts = Brdysts(self.brdysts.0 | event.brdysts.0);
        self.nrdysts = Nrdysts(self.nrdysts.0 | event.nrdysts.0);
        self.bempsts = Bempsts(self.bempsts.0 | event.bempsts.0);
        self.valid_pending |= event.valid_rising;
        if event.setup_valid {
            self.setup_valid = true;
            self.setup_packet = event.setup_packet;
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BusState {
    pub ep0_out_allocated: bool,
    pub ep0_in_allocated: bool,
    pub ep0_max_packet: u16,
    pub allocated_in_mask: u16,
    pub allocated_out_mask: u16,
    pub ep0_stalled: bool,
    pub stalled_in_mask: u16,
    pub stalled_out_mask: u16,
    pub ep0_last_setup_dir_out: bool,
    pub ep0_setup_pending: bool,
    pub ep0_setup_ready: bool,
    pub ep0_setup_packet: [u8; 8],
    pub ep0_short_in_waiting_status: bool,
    pub ep0_expect_status_out: bool,
    pub suspended: bool,
    pub pipe_bindings: [Option<PipeBinding>; 10],
    pub in_busy_mask: u16,
    pub brdyenb_shadow: u16,
    pub nrdyenb_shadow: u16,
    pub bempenb_shadow: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PipeBinding {
    pub ep_addr: EndpointAddress,
    pub ep_type: EndpointType,
    pub max_packet: u16,
}
