use alloc::format;
use core::arch::asm;
use core::ops::Sub;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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

    /// Get the [`Duration`] since a previous instant. This function panics if this instant was *before* the other instant.
    ///
    /// Note: `Instant` also implements `Sub`, so you can use the minus operator instead of this function.
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

    /// Check if this `Instant` is later than another `Instant`.
    ///
    /// Note: `Instant` also implements `Ord`, so you can use the comparison operators instead of this function.
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

impl Eq for Instant {}

impl PartialEq<Self> for Instant {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}

impl PartialOrd<Self> for Instant {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Instant {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.time.cmp(&other.time)
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
    cortex_m::interrupt::free(|_| {
        let global_time = GLOBAL_TIME.lock();
        let counter = RTC.lock().get_counter();

        let time = (*global_time + counter as u64) * PERIOD;

        send_bytes(format!("global: {} ctr: {}\n", *global_time, counter).as_bytes());

        time
    })
}

#[interrupt]
unsafe fn RTC0() {
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let mut rtc = RTC.lock();
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let mut global_time = GLOBAL_TIME.lock();

    if rtc.is_event_triggered(RtcInterrupt::Compare0) {
        *global_time += rtc.get_counter() as u64;

        rtc.clear_counter();
        while rtc.get_counter() != 0 {}

        rtc.reset_event(RtcInterrupt::Compare0);
        TIMER_FLAG.store(true, Ordering::SeqCst);
    }

}

/// Wait for the next interrupt configured by `set_interrupt_frequency`.
pub fn wait_for_next_tick() {
    while !TIMER_FLAG.load(Ordering::SeqCst) {
        cortex_m::asm::wfi();
    }
    TIMER_FLAG.store(false, Ordering::SeqCst);
}

/// Set this timer to interrupt at the given frequency.
/// The next interrupt will be after 1/hz seconds.
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
