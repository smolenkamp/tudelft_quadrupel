use core::time::Duration;

/// Delay for a number of CPU cycles. Very inaccurate
/// and hard to convert to an exact number of seconds
pub use cortex_m::asm::delay as assembly_delay;
use nrf51_hal::Rtc;
use nrf51_pac::RTC0;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;

/// A moment in time
#[derive(Debug, Copy, Clone)]
pub struct Instant {
    time: u64,
}

impl Instant {
    /// Return the current instant, i.e. the current time
    pub fn now() -> Self {
        Self {
            time: get_time_us()
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
            time: self.time + d.as_nanos() as u64
        }

    }

    pub fn is_later_than(self, other: Self) -> bool {
        self.time > other.time
    }

    /// Wait until this instant has passed. If this instant is a moment in the future,
    /// wait until that moment in the future.
    pub fn sleep_until(self) {
        // while the time we want to wait until is in the future (larger than now)
        while self.is_later_than(Instant::now()) {
            assembly_delay(10);
        }
    }
}


/// Wait for this duration.
pub fn sleep_for(d: Duration) {
    Instant::now().add_duration(d).sleep_until();
}

pub fn loop_at_freq(hz: u64, mut f: impl FnMut(Duration)) -> ! {
    let mut last = Instant::now();
    loop {
        let start = Instant::now();
        f(start.duration_since(last));
        let done = Instant::now();

        let expected_duration = Duration::from_nanos(1_000_000_000 / hz);
        let actual_duration = done.duration_since(start);

        if actual_duration < expected_duration {
            sleep_for(expected_duration - actual_duration);
        }

        last = start;
    }
}

static RTC: Mutex<OnceCell<(Rtc<RTC0>, u64)>> = Mutex::new(OnceCell::uninitialized());

fn get_time_us() -> u64 {
    let rtc = RTC.lock();
    let counter = rtc.0.get_counter();

    counter as u64 * rtc.1
}

/// TODO: better docs on what's going on here
/// TODO: maybe just use the largest frequency possible. This makes an
///       overflow occur ever 512 seconds. We need some way to detect this and
///       track it so our clock keeps working well.
pub(crate) fn initialize(clock: RTC0, clock_frequency: u8) {
    let mut rtc = RTC.lock();
    // calculate closest prescaler based on frequency
    let prescaler = (32_768 / clock_frequency as u32) + 1;

    // now work back to the frequency, convert that to a wavelength, and base
    // it on nanoseconds instead of seconds so we don't get a decimal number
    let period = ((prescaler as u64 + 1) * 1000 * 1000 * 1000) / 32_768u64;

    rtc.initialize((Rtc::new(clock, prescaler).unwrap(), period));

    rtc.0.enable_counter();
}
