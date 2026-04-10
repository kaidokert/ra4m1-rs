use core::{
    cell::{Cell, UnsafeCell},
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicU16, Ordering},
};

use embassy_hal_internal::interrupt::InterruptExt as _;
use embassy_hal_internal::{Peri, PeripheralType};

use crate::pac::usbfs::regs::Dcpctr;
use crate::pac::usbfs::vals::DcpctrPid;
use crate::{
    event_link::{IcuInterrupt as _, InterruptEvent},
    interrupt::{
        Interrupt,
        typelevel::{Binding, Handler as InterruptHandlerTrait, Interrupt as InterruptType},
    },
    pac, peripherals,
};

use super::{
    regs,
    types::{Error, UsbIrqEvent, UsbIrqLocalState},
};

const FORCE_RESET_DELAY_CYCLES: u32 = 480_000;

static USB_BRDYENB_SHADOW: AtomicU16 = AtomicU16::new(0);
static USB_NRDYENB_SHADOW: AtomicU16 = AtomicU16::new(0);
static USB_BEMPENB_SHADOW: AtomicU16 = AtomicU16::new(0);

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub force_reset_on_init: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            force_reset_on_init: true,
        }
    }
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

pub(super) trait SealedInstance {
    const INTERRUPT_EVENT: InterruptEvent;

    fn regs() -> pac::usbfs::Usbfs;
    fn event_pending() -> &'static AtomicBool;
    fn latched_event() -> &'static UnsafeCell<UsbIrqEvent>;
    fn irq_local_state() -> &'static UnsafeCell<UsbIrqLocalState>;
}

pub struct Driver<'d, I: Instance> {
    _peri: Peri<'d, I>,
    irq: Interrupt,
    attached: Cell<bool>,
    irq_enabled: Cell<bool>,
    _config: Config,
}

pub struct InterruptHandler<I: Instance> {
    _phantom: PhantomData<I>,
}

impl<'d, I: Instance + 'static> Driver<'d, I> {
    pub fn new<Int: InterruptType, H: InterruptHandlerTrait<Int>>(
        peri: Peri<'d, I>,
        _irqs: impl Binding<Int, H>,
        config: Config,
    ) -> Result<Self, Error> {
        let this = Self {
            _peri: peri,
            irq: Int::IRQ,
            attached: Cell::new(false),
            irq_enabled: Cell::new(false),
            _config: config,
        };

        init_usbfs_module();
        if usbfs_module_stopped() {
            return Err(Error::Busy);
        }
        configure_usbfs_pins();
        init_usbfs_registers(I::regs());

        if config.force_reset_on_init {
            this.force_reset()?;
        }

        Ok(this)
    }

    pub fn enable_interrupts(&self) {
        if self.irq_enabled.get() {
            return;
        }

        self.irq.disable();
        unsafe { self.irq.icu_enable(I::INTERRUPT_EVENT) };
        self.irq.icu_unpend();
        unsafe { self.irq.enable() };
        self.irq_enabled.set(true);
    }

    pub fn attach(&self) {
        let regs = I::regs();
        regs.syscfg().modify(|w| w.set_dprpu(true));
        self.attached.set(true);
    }

    pub fn detach(&self) {
        let regs = I::regs();
        regs.syscfg().modify(|w| w.set_dprpu(false));
        self.attached.set(false);
    }

    pub fn force_reset(&self) -> Result<(), Error> {
        let regs = I::regs();
        let mut syscfg = regs.syscfg().read();
        syscfg.set_dprpu(false);
        regs.syscfg().write_value(syscfg);

        let mut syscfg = regs.syscfg().read();
        syscfg.set_drpd(false);
        syscfg.set_dcfm(false);
        regs.syscfg().write_value(syscfg);

        regs.dcpcfg().write_value(Default::default());
        let dcpmaxp = regs.dcpmaxp().read().0;
        regs.dcpmaxp()
            .write_value(crate::pac::usbfs::regs::Dcpmaxp(dcpmaxp));
        let mut dcpctr = Dcpctr::default();
        dcpctr.set_sqclr(true);
        dcpctr.set_pid(DcpctrPid::_00);
        regs.dcpctr().write_value(dcpctr);

        cortex_m::asm::delay(FORCE_RESET_DELAY_CYCLES);
        self.attach();
        self.irq.icu_unpend();
        Ok(())
    }

    pub fn on_interrupt(&self) {
        capture_irq_event::<I>();
        self.irq.icu_unpend();
    }

    pub fn take_events(&self) -> UsbIrqEvent {
        I::event_pending().store(false, Ordering::Release);
        critical_section::with(|_| unsafe {
            let slot = &mut *I::latched_event().get();
            let event = *slot;
            *slot = UsbIrqEvent::default();
            event
        })
    }

    pub fn irq_pending(&self) -> bool {
        I::event_pending().load(Ordering::Acquire)
    }

    pub fn is_attached(&self) -> bool {
        self.attached.get()
    }

    pub unsafe fn on_interrupt_static<Int: InterruptType>() {
        capture_irq_event::<I>();
        Int::IRQ.icu_unpend();
    }

    pub(super) fn clear_capture_state_static() {
        I::event_pending().store(false, Ordering::Release);
        critical_section::with(|_| unsafe {
            *I::latched_event().get() = UsbIrqEvent::default();
            let local = &mut *I::irq_local_state().get();
            local.capture_valid_latched = false;
        });
    }
}

impl<I: Instance> InterruptHandler<I> {
    pub const fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

pub(super) fn set_brdyenb_shadow(mask: u16) {
    USB_BRDYENB_SHADOW.store(mask, Ordering::Release);
}

pub(super) fn set_nrdyenb_shadow(mask: u16) {
    USB_NRDYENB_SHADOW.store(mask, Ordering::Release);
}

pub(super) fn set_bempenb_shadow(mask: u16) {
    USB_BEMPENB_SHADOW.store(mask, Ordering::Release);
}

impl<I: Instance + 'static, Int: InterruptType> InterruptHandlerTrait<Int> for InterruptHandler<I> {
    unsafe fn on_interrupt() {
        unsafe { Driver::<'static, I>::on_interrupt_static::<Int>() };
    }
}

fn init_usbfs_module() {
    let system = pac::SYSTEM;
    let mstp = pac::MSTP;
    use crate::pac::system::vals::{Prc0, Prc1, Prkey};

    system.prcr().write(|w| {
        w.set_prkey(Prkey::PROTECT_KEY);
        w.set_prc0(Prc0::NotProtected);
        w.set_prc1(Prc1::NotProtected);
    });

    let mut mstpcrb = mstp.mstpcrb().read();
    mstpcrb.set_mstpb11(false);
    mstp.mstpcrb().write_value(mstpcrb);

    system.prcr().write(|w| {
        w.set_prkey(Prkey::PROTECT_KEY);
        w.set_prc0(Prc0::Protected);
        w.set_prc1(Prc1::Protected);
    });
}

fn usbfs_module_stopped() -> bool {
    pac::MSTP.mstpcrb().read().mstpb11()
}

fn configure_usbfs_pins() {
    const PFS_PMR: u32 = 1 << 16;
    const USBFS_PSEL: u32 = 0x13;

    let pfs = pac::PFS;
    let pmisc = pac::PMISC;

    pmisc
        .pwpr()
        .write_value(crate::pac::pmisc::regs::Pwpr(0x00));
    pmisc
        .pwpr()
        .write_value(crate::pac::pmisc::regs::Pwpr(0x40));

    pfs.pin(4, 7)
        .write_value(crate::pac::pfs::regs::PmnPfs((USBFS_PSEL << 24) | PFS_PMR));
    pfs.pin(9, 14)
        .write_value(crate::pac::pfs::regs::PmnPfs((USBFS_PSEL << 24) | PFS_PMR));
    pfs.pin(9, 15)
        .write_value(crate::pac::pfs::regs::PmnPfs((USBFS_PSEL << 24) | PFS_PMR));

    pmisc
        .pwpr()
        .write_value(crate::pac::pmisc::regs::Pwpr(0x80));
}

fn capture_irq_event<I: Instance>() {
    let regs = I::regs();
    let local = critical_section::with(|_| unsafe { &mut *I::irq_local_state().get() });
    local.capture_seqno = local.capture_seqno.wrapping_add(1);

    let intsts0 = regs.intsts0().read();
    let intenb0 = regs.intenb0().read();
    let ists0 = pac::usbfs::regs::Intsts0(intsts0.0 & intenb0.0);
    let intsts1 = regs.intsts1().read();
    let intenb1 = regs.intenb1().read();
    let ists1 = pac::usbfs::regs::Intsts1(intsts1.0 & intenb1.0);

    let ctrt_ctsq = if ists0.ctrt() {
        regs.intsts0().read().ctsq().to_bits() as u16
    } else {
        0
    };

    let valid_high = intsts0.valid();
    let valid_rising = if valid_high {
        if local.capture_valid_latched {
            false
        } else {
            local.capture_valid_latched = true;
            true
        }
    } else {
        local.capture_valid_latched = false;
        false
    };

    let capture_setup = valid_rising || matches!(ctrt_ctsq, 0x1 | 0x3 | 0x5);
    let setup_packet = if capture_setup {
        let usbreq = regs.usbreq().read().0.to_le_bytes();
        let usbval = regs.usbval().read().0.to_le_bytes();
        let usbindx = regs.usbindx().read().0.to_le_bytes();
        let usbleng = regs.usbleng().read().0.to_le_bytes();
        [
            usbreq[0], usbreq[1], usbval[0], usbval[1], usbindx[0], usbindx[1], usbleng[0],
            usbleng[1],
        ]
    } else {
        [0; 8]
    };

    let brdysts = pac::usbfs::regs::Brdysts(
        (regs.brdysts().read().0 & USB_BRDYENB_SHADOW.load(Ordering::Acquire))
            & regs::BRDY_BEMP_MASK,
    );
    let nrdysts = pac::usbfs::regs::Nrdysts(
        (regs.nrdysts().read().0 & USB_NRDYENB_SHADOW.load(Ordering::Acquire))
            & regs::BRDY_BEMP_MASK,
    );
    let bempsts = pac::usbfs::regs::Bempsts(
        (regs.bempsts().read().0 & USB_BEMPENB_SHADOW.load(Ordering::Acquire))
            & regs::BRDY_BEMP_MASK,
    );

    let mut captured_intsts0 = pac::usbfs::regs::Intsts0::default();
    captured_intsts0.set_dvsq(ists0.dvsq());
    captured_intsts0.set_resm(ists0.resm());
    captured_intsts0.set_sofr(ists0.sofr());
    captured_intsts0.set_dvst(ists0.dvst());
    captured_intsts0.set_ctrt(ists0.ctrt());

    let event = UsbIrqEvent {
        intsts0: captured_intsts0,
        intsts1: ists1,
        ctrt_ctsq,
        seqno: local.capture_seqno,
        valid_high,
        valid_rising,
        brdysts,
        nrdysts,
        bempsts,
        setup_valid: capture_setup,
        setup_packet,
    };

    if event.is_empty() {
        return;
    }

    critical_section::with(|_| unsafe {
        let slot = &mut *I::latched_event().get();
        slot.merge_from(event);
        I::event_pending().store(true, Ordering::Release);
    });
}

fn init_usbfs_registers(regs: pac::usbfs::Usbfs) {
    regs.syscfg().modify(|w| w.set_scke(true));

    while !regs.syscfg().read().scke() {}

    regs.syscfg().modify(|w| {
        w.set_dcfm(false);
        w.set_drpd(false);
        w.set_dprpu(false);
        w.set_usbe(true);
        w.set_scke(true);
    });

    regs.cfifosel().write(|w| w.set_mbw(false));
    regs.d0fifosel().write(|w| w.set_mbw(true));
    regs.d1fifosel().write(|w| w.set_mbw(false));
    regs.intsts0().write_value(Default::default());
    regs.intenb0().write(|w| {
        w.set_bempe(true);
        w.set_brdye(true);
        w.set_nrdye(true);
        w.set_vbse(true);
        w.set_dvse(true);
        w.set_ctre(true);
    });
    regs.intenb1().write_value(Default::default());
}

struct EventCell(UnsafeCell<UsbIrqEvent>);

unsafe impl Sync for EventCell {}

impl Instance for peripherals::USBFS {}

impl SealedInstance for peripherals::USBFS {
    const INTERRUPT_EVENT: InterruptEvent = InterruptEvent::UsbfsUsbi;

    fn regs() -> pac::usbfs::Usbfs {
        regs::usbfs()
    }

    fn event_pending() -> &'static AtomicBool {
        static PENDING: AtomicBool = AtomicBool::new(false);
        &PENDING
    }

    fn latched_event() -> &'static UnsafeCell<UsbIrqEvent> {
        static LATCHED: EventCell = EventCell(UnsafeCell::new(UsbIrqEvent {
            intsts0: pac::usbfs::regs::Intsts0(0),
            intsts1: pac::usbfs::regs::Intsts1(0),
            ctrt_ctsq: 0,
            seqno: 0,
            valid_high: false,
            valid_rising: false,
            brdysts: pac::usbfs::regs::Brdysts(0),
            nrdysts: pac::usbfs::regs::Nrdysts(0),
            bempsts: pac::usbfs::regs::Bempsts(0),
            setup_valid: false,
            setup_packet: [0; 8],
        }));
        &LATCHED.0
    }

    fn irq_local_state() -> &'static UnsafeCell<UsbIrqLocalState> {
        struct LocalCell(UnsafeCell<UsbIrqLocalState>);
        unsafe impl Sync for LocalCell {}
        static LOCAL: LocalCell = LocalCell(UnsafeCell::new(UsbIrqLocalState {
            capture_valid_latched: false,
            capture_seqno: 0,
        }));
        &LOCAL.0
    }
}
