//! Universal Serial Bus Full-Speed device support (`USBFS`).
//!
//! This module is scaffolded around the proven RA4M1 USBFS implementation used
//! on EK-RA4M1. The long-term design keeps the hardware-facing USBFS driver in
//! the HAL while the `usb-device` integration layer remains above it.
#![allow(missing_docs)]

mod bus;
mod driver;
mod regs;
mod types;

pub use self::{
    bus::Bus,
    driver::{Config, Driver, Instance, InterruptHandler},
    types::{BusState, Error, PipeBinding, UsbEventSnapshot, UsbIrqEvent, UsbIrqLocalState},
};
