//! `usb_cdc` demonstrates a USB CDC ACM serial port using `usbd-serial`.
//!
//! The example uses the same proven USB bring-up sequence as the working
//! `ra4_r` test path, then echoes received bytes back to the host.

#![no_std]
#![no_main]
#![warn(missing_docs)]

use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use cortex_m::asm;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use panic_probe as _;
use ra4_hal::pac;
use ra4_hal::{bind_interrupts, peripherals::USBFS, usb};
#[allow(unused)]
use ra4_hal::{debug, error, info, trace, warn};
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    prelude::UsbDeviceState,
};
use usbd_serial::SerialPort;

bind_interrupts!(struct Irqs {
    IEL5 => usb::InterruptHandler<USBFS>;
});

struct Shared<T>(UnsafeCell<T>);

unsafe impl<T> Sync for Shared<T> {}

type HalBus = usb::Bus<'static, USBFS>;
type HalDevice = usb_device::device::UsbDevice<'static, HalBus>;
type HalSerial = SerialPort<'static, HalBus>;

static USB_BUS_ALLOC: Shared<Option<UsbBusAllocator<HalBus>>> = Shared(UnsafeCell::new(None));
static USB_DEVICE: Shared<Option<HalDevice>> = Shared(UnsafeCell::new(None));
static USB_SERIAL: Shared<Option<HalSerial>> = Shared(UnsafeCell::new(None));

static USB_LAST_STATE: AtomicU8 = AtomicU8::new(0xff);
static DTR_ACTIVE: AtomicBool = AtomicBool::new(false);

fn encode_state(state: UsbDeviceState) -> u8 {
    match state {
        UsbDeviceState::Default => 0,
        UsbDeviceState::Addressed => 1,
        UsbDeviceState::Configured => 2,
        UsbDeviceState::Suspend => 3,
    }
}

fn decode_state(code: u8) -> Option<UsbDeviceState> {
    match code {
        0 => Some(UsbDeviceState::Default),
        1 => Some(UsbDeviceState::Addressed),
        2 => Some(UsbDeviceState::Configured),
        3 => Some(UsbDeviceState::Suspend),
        _ => None,
    }
}

fn unlock_prcr() {
    pac::SYSTEM
        .prcr()
        .write_value(pac::system::regs::Prcr(0xA503));
}

fn lock_prcr() {
    pac::SYSTEM
        .prcr()
        .write_value(pac::system::regs::Prcr(0xA500));
}

fn quickstart_sckdivcr() -> u32 {
    (1u32) << 28 | (0u32) << 24 | (1u32) << 16 | (0u32) << 12 | (1u32) << 8 | (0u32) << 4
}

fn boot_pause() {
    asm::delay(4_800_000);
}

fn attach_settle_pause() {
    asm::delay(480_000);
}

fn setup_pll_48mhz() {
    let system = pac::SYSTEM;

    unlock_prcr();

    system
        .hococr()
        .modify(|r| r.set_hcstp(pac::system::vals::Hcstp::Start));
    let mut hoco_ready = false;
    for _ in 0..100_000 {
        if system.oscsf().read().hocosf() {
            hoco_ready = true;
            break;
        }
    }
    if !hoco_ready {
        lock_prcr();
        panic!("HOCO timeout");
    }

    system
        .opccr()
        .modify(|r| r.set_opcm(pac::system::vals::Opcm::HighSpeed));
    let mut opcm_ready = false;
    for _ in 0..100_000 {
        let opccr = system.opccr().read();
        if !opccr.opcmtsf() && opccr.opcm().to_bits() == 0 {
            opcm_ready = true;
            break;
        }
    }
    if !opcm_ready {
        lock_prcr();
        panic!("OPCM timeout");
    }

    if system.mosccr().read().0 != 0 {
        while system.oscsf().read().moscsf() {}

        system.momcr().modify(|r| {
            r.set_mosel(false);
            r.set_modrv1(false);
        });
        system
            .moscwtcr()
            .write_value(pac::system::regs::Moscwtcr(0x09));
        system.mosccr().modify(|r| r.set_mostp(false));
    }
    let mut mosc_ready = false;
    for _ in 0..5_000_000 {
        if system.oscsf().read().moscsf() {
            mosc_ready = true;
            break;
        }
    }
    if !mosc_ready {
        lock_prcr();
        panic!("MOSC timeout");
    }

    if system.pllcr().read().0 != 0 {
        system
            .pllccr2()
            .write_value(pac::system::regs::Pllccr2(0x47));
        asm::delay(64);
        system.pllcr().write_value(pac::system::regs::Pllcr(0x00));
    }
    let mut pll_ready = false;
    for _ in 0..100_000 {
        if system.oscsf().read().pllsf() {
            pll_ready = true;
            break;
        }
    }
    if !pll_ready {
        lock_prcr();
        panic!("PLL timeout");
    }

    system.memwait().modify(|r| r.set_memwait(true));
    system
        .sckdivcr()
        .write_value(pac::system::regs::Sckdivcr(quickstart_sckdivcr()));
    system
        .sckscr()
        .modify(|r| r.set_cksel(pac::system::vals::Cksel::Pll));

    lock_prcr();
}

fn log_state(state: UsbDeviceState) {
    match state {
        UsbDeviceState::Default => info!("usb state=Default"),
        UsbDeviceState::Addressed => info!("usb state=Addressed"),
        UsbDeviceState::Configured => info!("usb state=Configured"),
        UsbDeviceState::Suspend => info!("usb state=Suspend"),
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = ra4_hal::init();
    setup_pll_48mhz();
    boot_pause();

    let driver = usb::Driver::new(
        p.USBFS,
        Irqs,
        usb::Config {
            force_reset_on_init: false,
        },
    )
    .unwrap();
    let bus = usb::Bus::new(driver);
    unsafe {
        *USB_BUS_ALLOC.0.get() = Some(UsbBusAllocator::new(bus));
    }

    boot_pause();
    let usb_bus = unsafe { (&*USB_BUS_ALLOC.0.get()).as_ref().unwrap() };
    let serial = SerialPort::new(usb_bus);
    unsafe {
        *USB_SERIAL.0.get() = Some(serial);
    }

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4d31))
        .composite_with_iads()
        .max_packet_size_0(64)
        .unwrap()
        .strings(&[StringDescriptors::new(usb_device::LangID::EN_US)
            .manufacturer("ra4-hal")
            .product("ra4-examples usb_cdc")
            .serial_number("ra4-examples-usb-cdc")])
        .unwrap()
        .build();

    unsafe {
        *USB_DEVICE.0.get() = Some(usb_dev);
        (&*USB_DEVICE.0.get())
            .as_ref()
            .unwrap()
            .bus()
            .driver()
            .enable_interrupts();
        (&mut *USB_DEVICE.0.get())
            .as_mut()
            .unwrap()
            .force_reset()
            .expect("force reset must succeed");
    }
    attach_settle_pause();

    let mut last_state = UsbDeviceState::Default;
    let mut last_dtr = false;
    let mut echo_buf = [0u8; 64];
    let mut echo_len = 0usize;
    let mut echo_off = 0usize;

    loop {
        if let (Some(usb_dev), Some(serial)) =
            (unsafe { (&mut *USB_DEVICE.0.get()).as_mut() }, unsafe {
                (&mut *USB_SERIAL.0.get()).as_mut()
            })
        {
            usb_dev.poll(&mut [serial]);
            USB_LAST_STATE.store(encode_state(usb_dev.state()), Ordering::Relaxed);
            DTR_ACTIVE.store(serial.dtr(), Ordering::Relaxed);
        }

        if let Some(state) = decode_state(USB_LAST_STATE.load(Ordering::Relaxed)) {
            if state != last_state {
                last_state = state;
                log_state(last_state);
            }
        }

        let dtr_active = DTR_ACTIVE.load(Ordering::Relaxed);
        if dtr_active != last_dtr {
            last_dtr = dtr_active;
            info!("dtr={}", dtr_active);
        }

        if let Some(serial) = unsafe { (&mut *USB_SERIAL.0.get()).as_mut() } {
            if echo_len == 0 {
                if dtr_active {
                    match serial.read(&mut echo_buf) {
                        Ok(count) if count > 0 => {
                            echo_len = count;
                            echo_off = 0;
                        }
                        _ => {}
                    }
                }
            } else {
                match serial.write(&echo_buf[echo_off..echo_len]) {
                    Ok(written) if written > 0 => {
                        echo_off += written;
                        if echo_off == echo_len {
                            echo_len = 0;
                            echo_off = 0;
                        }
                    }
                    Ok(_) => {}
                    Err(usb_device::UsbError::WouldBlock) => {}
                    Err(_) => {
                        echo_len = 0;
                        echo_off = 0;
                    }
                }
            }
        }

        asm::nop();
    }
}
