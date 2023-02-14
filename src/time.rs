use alloc::format;
use core::arch::asm;
use core::ops::Sub;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use nrf51_pac::interrupt;

use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
/// Delay for a number of CPU cycles. Very inaccurate
/// and hard to convert to an exact number of seconds
pub use cortex_m::asm::delay as assembly_delay;
use cortex_m::peripheral::NVIC;
use nrf51_hal::Rtc;
use nrf51_hal::rtc::{RtcCompareReg, RtcInterrupt};
use nrf51_pac::RTC0;
use crate::led::Led::Green;
use crate::uart::send_bytes;

/// A moment in time
#[derive(Debug, Copy, Clone)]
pub struct Instant {
    time: u64,
}

impl Instant {
    /// Return the current instant, i.e. the current time
    pub fn now() -> Self {
        Self {
            time: get_time_ns(),
        }
    }

    /// Get the [`Duration`] since a previous instant
    pub fn duration_since(self, other: Self) -> Duration {
        assert!(self.time >= other.time);
        Duration::from_nanos(self.time - other.time)
    }

    /// Adds a duration to this instant, producing a new instant in the future
    pub fn add_duration(self, d: Duration) -> Self {
        Self {
            time: self.time + d.as_nanos() as u64,
        }
    }

    pub fn is_later_than(self, other: Self) -> bool {
        self.time > other.time
    }
}

impl Sub<Self> for Instant {
    type Output = Duration;

    /// Get the [`Duration`] since a previous instant (rhs)
    fn sub(self, rhs: Self) -> Self::Output {
        self.duration_since(rhs)
    }
}

// RTC0 is used for measuring absolute instants
static RTC: Mutex<OnceCell<Rtc<RTC0>>> = Mutex::new(OnceCell::uninitialized());

/// take the highest prescaler
/// NOTE: change the period below when changing the prescaler
const PRESCALER: u32 = 0;
/// giving a period of this many nanoseconds
const PERIOD: u64 = 30517;

/// is set to true when the timer interrupt has gone off.
/// Used to wait on the timer interrupt in [`wait_for_interrupt`]
static TIMER_FLAG: AtomicBool = AtomicBool::new(false);

/// Global time in magic timer units ([`PERIOD`]) since timers started
/// SAFETY: only changed within timer interrupt. Safe to read at all times
static GLOBAL_TIME: Mutex<u64> = Mutex::new(0);

pub(crate) fn initialize(clock_instance: RTC0, nvic: &mut NVIC) {
    let mut rtc = RTC.lock();

    rtc.initialize(Rtc::new(clock_instance, PRESCALER).unwrap());
    rtc.enable_event(RtcInterrupt::Compare0);
    rtc.enable_interrupt(RtcInterrupt::Compare0, Some(nvic));
    rtc.enable_counter();
}

// get the current global time in nanoseconds. The precision is not necessarily in single nanoseconds
fn get_time_ns() -> u64 {
    let global_time = GLOBAL_TIME.lock();
    let counter = RTC.lock().get_counter();
    send_bytes(format!("global: {} ctr: {}\n", *global_time, counter).as_bytes());

    (*global_time + counter as u64) * PERIOD
}

#[interrupt]
unsafe fn RTC0() {
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let rtc = RTC.no_critical_section_lock();
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let global_time = GLOBAL_TIME.no_critical_section_lock();

    if rtc.is_event_triggered(RtcInterrupt::Compare0) {
        *global_time += rtc.get_counter() as u64;

        rtc.clear_counter();
        rtc.reset_event(RtcInterrupt::Compare0);
        TIMER_FLAG.store(true, Ordering::SeqCst);
    }

}

pub fn wait_for_next_tick() {
    while !TIMER_FLAG.load(Ordering::SeqCst) {
        cortex_m::asm::wfi();
    }
    TIMER_FLAG.store(false, Ordering::SeqCst);
}

pub fn set_tick_frequency(hz: u64) {
    let mut rtc = RTC.lock();

    let counter_setting = (1_000_000_000 / hz) / PERIOD;
    assert!(counter_setting < (1 << 24));

    rtc.set_compare(RtcCompareReg::Compare0, counter_setting as u32).unwrap();
    rtc.clear_counter();
}

/// Delay the program for a time using assembly instructions.
/// Testing shows this overshoots by ~5%, which is the closest that is possible without undershooting.
#[allow(unused_assignments)]
pub fn delay_us_assembly(mut number_of_us: u32) {
    unsafe {
        asm!(
        "1:",
        "subs {}, #1",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "bne 1b",
        inout(reg) number_of_us,
        options(nomem, nostack)
        )
    }
}

/// Delay the program for a time using assembly instructions.
/// Testing shows this overshoots by ~5%, which is the closest that is possible without undershooting.
pub fn delay_ms_assembly(number_of_ms: u32) {
    for _ in 0..number_of_ms {
        delay_us_assembly(999);
    }
}
