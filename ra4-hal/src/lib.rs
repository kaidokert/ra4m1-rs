#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

//! ## Feature flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]

// This needs to come first so the macros are visible everywhere else.
#[doc(hidden)]
pub mod fmt;

pub mod adc;
pub mod crc;
pub mod dac;
pub mod dmac;
pub mod dtc;
pub mod event_link;
pub mod gpio;
pub mod i2c;
pub mod mcu_info;
pub mod module_stop;
pub mod osm;
pub mod pwm;
pub mod qdec;
#[cfg(feature = "_enable-rtc-beware-of-dragons")]
pub mod rtc;
pub mod timer;
pub mod usb;
pub mod watchdog;
// pub mod sce5;
pub mod spi;
#[cfg(feature = "time-driver")]
pub mod time_driver;
pub mod uart;
pub mod write_protect;

// Re-exports
#[cfg(feature = "chrono")]
pub use chrono;
#[cfg(feature = "unstable-pac")]
pub use ra4m1_ctpac as pac;

use cfg_if::cfg_if;
use cortex_m::asm;
use pac::system::vals::{Cksel, Fck, Hcfrq1, Hcstp, Ick, Opcm, Pcka, Pckb, Pckc, Pckd};
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use ra4m1_ctpac as pac;

use crate::{mcu_info::McuInfo, write_protect::ProtectedPeripheral as _};

/// System clock configuration
///
/// # Notes
///
/// Assumes that the system clock source `ICLK` = `HOCO`.
/// Possible sources (§ 48.3.2, Table 8.2):
/// * `MOSC` 1–20 MHz depending on `Vcc`, user supplied external oscillator
/// * `SOSC` 32.768 kHz user supplied external oscillator
/// * `HOCO` ±1% @ 48,64 MHz (±1.5% extreme cold, ±2% extreme heat)
/// * `MOCO` 8 MHz, ±15%
/// * `LOCO` 32.768 kHz ±15%
/// * `PLL` driven by `MOSC`, output 24–64 MHz
pub struct ClockConfig {
    /// System clock frequency (`ICLK`).
    ///
    /// Supplies: `CPU`, `DMAC`, [`DTC`](module@dtc), `SRAM`, and the flash memory.
    pub system: u32,

    /// Flash interface clock (`FCLK`).
    pub flash: u32,

    /// Peripheral Clock "A" (`PCLKA`).
    ///
    /// Supplies:
    /// [`CRC`](module@crc),
    /// `SCE5`,
    /// [`SCI`](module@uart),
    /// [`SPI`](module@spi),
    /// and the [`GPT`](module@timer) bus clock.
    pub peripheral_a: u32,

    /// Peripheral Clock "B" (`PCLKB`).
    ///
    /// Supplies:
    /// `ACMPLP`,
    /// [`ADC14`](module@adc),
    /// `AGT`,
    /// `CAC`,
    /// `CAN`,
    /// `CTSU`,
    /// [`DAC12`](module@dac),
    /// `DOC`,
    /// [`ELC`](module@event_link),
    /// [`IIC`](module@i2c),
    /// `IWDT`,
    /// `KINT`,
    /// `POEG`,
    /// [`PORT`](module@gpio) (I/O Ports),
    /// `RTC`,
    /// `SLCDC`,
    /// `SSIE`,
    /// `USBFS`,
    /// and [`WDT`](module@watchdog::wdt).
    pub peripheral_b: u32,

    /// Peripheral Clock "C" (`PCLKC`).
    ///
    /// Supplies [`ADC14`](module@adc) conversion clock.
    pub peripheral_c: u32,

    /// Peripheral Clock "D" (`PCLKD`).
    ///
    /// Supplies [`GPT`](module@timer) count clock.
    pub peripheral_d: u32,
}

/// Coarse indication of why the processor reset.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(unused)]
pub enum ResetCause {
    /// Power was turned on.
    PowerOn,

    /// Low voltage monitor 0, 1, or 2 tripped.
    LowVoltage,

    /// Watchdog or independent watchdog.
    Watchdog,

    /// Bus error, parity error, or ECC error.
    HadwareError,

    /// Stack pointer error.
    StackPointer,

    /// Software reset requested.
    SoftwareReset,

    /// Should never be here.
    Unknown,
}

/// Initializes the MCU.
///
/// Currently limited to setting up the clocks and the time driver.
///
/// # Returns
///
/// The available peripherals as a [`Peripherals`] struct.
pub fn init() -> Peripherals {
    critical_section::with(|cs| {
        let system = pac::SYSTEM;
        #[cfg(feature = "strict-assert")]
        let fmifrt_base = pac::FMIFRT_BASE;

        debug!("Starting board init");

        #[cfg(feature = "diag")]
        {
            let reset_status_0 = system.rstsr0().read();
            let reset_status_1 = system.rstsr1().read();

            let reset_cause: ResetCause;
            if reset_status_0.porf() {
                reset_cause = ResetCause::PowerOn;
            } else if reset_status_0.lvd0rf() || reset_status_0.lvd1rf() || reset_status_0.lvd2rf()
            {
                reset_cause = ResetCause::LowVoltage;
            } else if reset_status_1.wdtrf() || reset_status_1.iwdtrf() {
                reset_cause = ResetCause::Watchdog;
            } else if reset_status_1.rperf()
                || reset_status_1.reerf()
                || reset_status_1.bussrf()
                || reset_status_1.busmrf()
            {
                reset_cause = ResetCause::HadwareError;
            } else if reset_status_1.sperf() {
                reset_cause = ResetCause::StackPointer;
            } else if reset_status_1.swrf() {
                reset_cause = ResetCause::SoftwareReset;
            } else {
                reset_cause = ResetCause::Unknown;
            }

            debug!("Reset reason: {}", reset_cause);
        }

        // Sanity check.  The manual states that this should be fixed.
        #[cfg(feature = "strict-assert")]
        assert_eq!(
            pac::fmifrt_base::vals::ExpectedBase::RA4M1.to_bits(),
            fmifrt_base.base().read().base()
        );

        let mcu_info = McuInfo::info();

        #[cfg(feature = "defmt")]
        mcu_info.print_info();

        // Check if the crate was configured correctly
        mcu_info.validate_pin_count();

        let osm = pac::OSM;
        debug!("OFS0: {}", osm.ofs0().read());
        debug!("OFS1: {}", osm.ofs1().read());

        trace!(
            "HOCO: wait={}, status={}",
            system.hocowtcr().read(),
            system.hococr().read()
        );

        system.protected_write(|| {
            if system.hococr().read().hcstp() != Hcstp::Start {
                warn!("HOCO: Not running, attempt to start.");
                system.hococr().write(|w| w.set_hcstp(Hcstp::Start));
            }

            // TODO: Add feature knobs to set HOCO frequency after boot?
            // #[cfg(feature = "frequency-after-reset")]
            // warn!("HOCO: Forcibly setting frequency={}",);
            // system.hococr2().write(|w| {
            //     w.set_hcfrqw(Hcfrq1::_48mhz);
            // });

            let hoco_freq = system.hococr2().read().hcfrqw();
            debug!(
                "HOCO: frequency={}, status={}",
                hoco_freq,
                system.hococr().read().hcstp()
            );

            cfg_if! {
                if #[cfg(feature = "hoco_32mhz")] {
                    let target_freq = Hcfrq1::_32mhz;
                } else if #[cfg(feature = "hoco_48mhz")] {
                    let target_freq = Hcfrq1::_48mhz;
                } else if #[cfg(feature = "hoco_64mhz")] {
                    let target_freq = Hcfrq1::_64mhz;
                } else {
                    compile_error!()
                }
            }

            if hoco_freq != target_freq {
                warn!("HOCO: expected={}, actual={}", hoco_freq, target_freq);
            }

            // High speed mode needed for ICLK > 12 MHz.  Currently there are no features to select
            // ICLK <= 12 MHz so just enable high speed mode unconditionally.
            trace!("Setting high speed mode on");
            system.opccr().write(|w| w.set_opcm(Opcm::HighSpeed));

            while system.opccr().read().opcmtsf() {
                asm::nop();
            }

            // Wait states needed for ICLK > 32 MHz
            if hoco_freq == Hcfrq1::_48mhz || hoco_freq == Hcfrq1::_64mhz {
                trace!("Setting SYSTEM_MEMWAIT to 1");
                system.memwait().write(|w| w.set_memwait(true));
            }

            // Use HOCO as clock source
            system.sckscr().write(|w| w.set_cksel(Cksel::Hoco));
            debug!("SYSTEM: ClkSource: {}", system.sckscr().read().cksel());

            #[cfg(feature = "cache")]
            {
                let fcache = pac::FCACHE;
                fcache.fcacheiv().write(|r| r.set_fcacheiv(true));

                while fcache.fcacheiv().read().fcacheiv() {
                    asm::nop();
                }

                fcache.fcachee().write(|r| r.set_fcacheen(true));

                info!("SYSTEM: fcache enabled");
            }
            #[cfg(not(feature = "cache"))]
            {
                let fcache = pac::FCACHE;
                fcache.fcacheiv().write(|r| r.set_fcacheiv(true));
                fcache.fcachee().write(|r| r.set_fcacheen(false));
                trace!("SYSTEM: fcache disabled");
            }

            // Max frequencies Table 8.2, p130
            // ICLK = 48 MHz
            // FCLK = 32 MHz
            // PCKLA = 48 MHz
            // PCLKB = 32 MHz
            // PCLKC = 64 MHz
            // PCLKD = 64 MHz
            match hoco_freq {
                Hcfrq1::_32mhz => system.sckdivcr().modify(|w| {
                    // 32 MHz
                    w.set_ick(Ick::DIV_1);
                    // 32 MHz
                    w.set_fck(Fck::DIV_1);
                    // 32 MHz
                    w.set_pcka(Pcka::DIV_1);
                    // 32 MHz
                    w.set_pckb(Pckb::DIV_1);
                    // 32 MHz
                    w.set_pckc(Pckc::DIV_1);
                    // 32 MHz
                    w.set_pckd(Pckd::DIV_1);
                }),
                Hcfrq1::_48mhz => system.sckdivcr().modify(|w| {
                    // 48 MHz
                    w.set_ick(Ick::DIV_1);
                    // 24 MHz
                    w.set_fck(Fck::DIV_2);
                    // 48 MHz
                    w.set_pcka(Pcka::DIV_1);
                    // 24 MHz
                    w.set_pckb(Pckb::DIV_2);
                    // 48 MHz
                    w.set_pckc(Pckc::DIV_1);
                    // 48 MHz
                    w.set_pckd(Pckd::DIV_1);
                }),
                // Faster peripheral clocks, slower CPU clock
                Hcfrq1::_64mhz => system.sckdivcr().modify(|w| {
                    // 32 MHz
                    w.set_ick(Ick::DIV_2);
                    // 32 MHz
                    w.set_fck(Fck::DIV_2);
                    // 32 MHz
                    w.set_pcka(Pcka::DIV_2);
                    // 32 MHz
                    w.set_pckb(Pckb::DIV_2);
                    // 64 MHz
                    w.set_pckc(Pckc::DIV_1);
                    // 64 MHz
                    w.set_pckd(Pckd::DIV_1);
                }),
                _ => unimplemented!(),
            }
        });
        debug!("Finished board init");

        print_clock_config();

        let p = Peripherals::take_with_cs(cs);

        #[cfg(feature = "time-driver")]
        time_driver::init();
        event_link::init();
        dtc::init();
        dmac::init();

        p
    })
}

/// Returns the current clock configuration.
pub fn clock_config() -> ClockConfig {
    let system = pac::SYSTEM;

    let hoco_freq = system.hococr2().read().hcfrqw();
    let hoco_freq = match hoco_freq {
        Hcfrq1::_24mhz => 24_000_000_u32,
        Hcfrq1::_32mhz => 32_000_000_u32,
        Hcfrq1::_48mhz => 48_000_000_u32,
        Hcfrq1::_64mhz => 64_000_000_u32,
        _ => unimplemented!(),
    };

    let prescaler = system.sckdivcr().read();

    let system = match prescaler.ick() {
        Ick::DIV_1 => hoco_freq,
        Ick::DIV_2 => hoco_freq / 2,
        Ick::DIV_4 => hoco_freq / 4,
        Ick::DIV_8 => hoco_freq / 8,
        Ick::DIV_16 => hoco_freq / 16,
        Ick::DIV_32 => hoco_freq / 32,
        Ick::DIV_64 => hoco_freq / 64,
        Ick::_RESERVED_7 => unimplemented!("Invalid sckdivcr.ick"),
    };

    let flash = match prescaler.fck() {
        Fck::DIV_1 => hoco_freq,
        Fck::DIV_2 => hoco_freq / 2,
        Fck::DIV_4 => hoco_freq / 4,
        Fck::DIV_8 => hoco_freq / 8,
        Fck::DIV_16 => hoco_freq / 16,
        Fck::DIV_32 => hoco_freq / 32,
        Fck::DIV_64 => hoco_freq / 64,
        Fck::_RESERVED_7 => unimplemented!("Invalid sckdivcr.fck"),
    };

    let peripheral_a = match prescaler.pcka() {
        Pcka::DIV_1 => hoco_freq,
        Pcka::DIV_2 => hoco_freq / 2,
        Pcka::DIV_4 => hoco_freq / 4,
        Pcka::DIV_8 => hoco_freq / 8,
        Pcka::DIV_16 => hoco_freq / 16,
        Pcka::DIV_32 => hoco_freq / 32,
        Pcka::DIV_64 => hoco_freq / 64,
        Pcka::_RESERVED_7 => unimplemented!("Invalid sckdivcr.pcka"),
    };

    let peripheral_b = match prescaler.pckb() {
        Pckb::DIV_1 => hoco_freq,
        Pckb::DIV_2 => hoco_freq / 2,
        Pckb::DIV_4 => hoco_freq / 4,
        Pckb::DIV_8 => hoco_freq / 8,
        Pckb::DIV_16 => hoco_freq / 16,
        Pckb::DIV_32 => hoco_freq / 32,
        Pckb::DIV_64 => hoco_freq / 64,
        Pckb::_RESERVED_7 => unimplemented!("Invalid sckdivcr.pckb"),
    };

    let peripheral_c = match prescaler.pckc() {
        Pckc::DIV_1 => hoco_freq,
        Pckc::DIV_2 => hoco_freq / 2,
        Pckc::DIV_4 => hoco_freq / 4,
        Pckc::DIV_8 => hoco_freq / 8,
        Pckc::DIV_16 => hoco_freq / 16,
        Pckc::DIV_32 => hoco_freq / 32,
        Pckc::DIV_64 => hoco_freq / 64,
        Pckc::_RESERVED_7 => unimplemented!("Invalid sckdivcr.pckc"),
    };

    let peripheral_d = match prescaler.pckd() {
        Pckd::DIV_1 => hoco_freq,
        Pckd::DIV_2 => hoco_freq / 2,
        Pckd::DIV_4 => hoco_freq / 4,
        Pckd::DIV_8 => hoco_freq / 8,
        Pckd::DIV_16 => hoco_freq / 16,
        Pckd::DIV_32 => hoco_freq / 32,
        Pckd::DIV_64 => hoco_freq / 64,
        Pckd::_RESERVED_7 => unimplemented!("Invalid sckdivcr.pckd"),
    };

    ClockConfig {
        system,
        flash,
        peripheral_a,
        peripheral_b,
        peripheral_c,
        peripheral_d,
    }
}

/// Logs the current clock configuration at the `debug` level.
///
/// # Notes
///
/// No-op unless the `defmt` feature is enabled.
#[inline]
pub fn print_clock_config() {
    #[cfg(feature = "defmt")]
    {
        let clock_config = clock_config();
        let ick_freq = clock_config.system / 1_000_000;
        let fck_freq = clock_config.flash / 1_000_000;
        let pck_a = clock_config.peripheral_a / 1_000_000;
        let pck_b = clock_config.peripheral_b / 1_000_000;
        let pck_c = clock_config.peripheral_c / 1_000_000;
        let pck_d = clock_config.peripheral_d / 1_000_000;

        let system = pac::SYSTEM;
        let cksel = system.sckscr().read().cksel();

        info!(
            "SYSTEM: SRC: {}, ICLK: {} MHz, FCLK: {} MHz, PCLKA: {} MHz, PCLKB: {} MHz, PCLKC: {} MHz, PCLKD: {} MHz",
            cksel, ick_freq, fck_freq, pck_a, pck_b, pck_c, pck_d
        );
    }
}

// NOTE: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
/// Macro to bind interrupts to interrupt handlers.
///
/// For example:
///
/// ```rust,ignore
/// use ra4_hal::{bind_interrupts, peripherals::SCI1, uart};
///
/// bind_interrupts!(struct Irqs {
///     IEL2 => uart::RxInterruptHandler<SCI1>;
///     IEL3 => uart::TxInterruptHandler<SCI1>;
///     IEL4 => uart::TeInterruptHandler<SCI1>;
/// });
///```
///
/// Any interrupt `IEL2..=IEL31` can be assigned to any one handler.
/// Note that `IEL0` and `IEL1` are used by the [time driver](crate::time_driver) and are unavailable for general use.
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$outer:meta])* $vis:vis struct $name:ident {
        $(
            $(#[doc = $doc:literal])*
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$outer])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_irq)])?
            $(#[doc = $doc])*
            unsafe extern "C" fn $irq() {
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();

                    )*
                }
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

include!(concat!(env!("OUT_DIR"), "/pin_traits.rs"));
include!(concat!(env!("OUT_DIR"), "/interrupts.rs"));
include!(concat!(env!("OUT_DIR"), "/peripherals.rs"));
include!(concat!(env!("OUT_DIR"), "/module_stops.rs"));

#[cfg(not(feature = "skip-osm"))]
mod _osm_config {
    use crate::osm::{ofs0::Ofs0, ofs1::Ofs1, sec_mpu::SecurityMpu};

    // Option Function Select Register 0
    #[unsafe(no_mangle)]
    #[unsafe(link_section = ".ofs0")]
    static OFS0: Ofs0 = Ofs0::default();

    // Option Function Select Register 1
    #[unsafe(no_mangle)]
    #[unsafe(link_section = ".ofs1")]
    static OFS1: Ofs1 = Ofs1::default();

    // Security MPU
    #[unsafe(no_mangle)]
    #[unsafe(link_section = ".sec_mpu")]
    static SEC_MPU: SecurityMpu = SecurityMpu::disabled();
}
