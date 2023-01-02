use crate::led::Led::Red;
use crate::mutex::Mutex;
use crate::time::assembly_delay;
use crate::{barometer, battery, led, motor, mpu, time, twi, uart};
use alloc_cortex_m::CortexMHeap;
use core::mem::MaybeUninit;
use nrf51_pac::Peripherals;
static INITIALIZED: Mutex<bool> = Mutex::new(false);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// Initialize the drone board. heap_memory should be pointer to
/// statically allocated memory. Care should be taken that the mutable reference
/// given here *really* is the only mutable reference to that area of memory.
/// That should of course be guaranteed by the fact that the reference is mutable, but
/// you need unsafe code to create a mutable reference to static memory. Care should be taken
/// that only one such reference is unsafely created.
///
/// clock_frequency determines the minimum delay / sleep time that can be set up. For example,
/// if the clock frequency is 100 hertz, sleeping for 1ms is not possible. The smallest sleep
/// time achievable with a 100 hertz clock is 10ms.
pub fn initialize(heap_memory: &'static mut [MaybeUninit<u8>], clock_frequency: u8) {
    // Allow time for PC to start up. The drone board starts running code immediately after upload,
    // but at that time the PC may not be listening on UART etc.
    assembly_delay(2500000);

    // keep this guard around until the end of the function (so interrupts stay off)
    let mut guard = INITIALIZED.lock();
    if *guard {
        panic!("ALREADY INITIALIZED");
    }
    *guard = true;

    // unwrap: will never panic because this function can only be called once (see the guard above)
    let mut nrf51_peripherals = Peripherals::take().unwrap();
    let mut cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();

    // Safety: `init` is safe when
    // * only called once --> the global INITIALIZED flag is set, and we panic above if called twice
    // * Heap is not empty, see the assert
    assert!(!heap_memory.is_empty());
    unsafe { ALLOCATOR.init(heap_memory.as_ptr().addr(), heap_memory.len()) }

    let gpio = nrf51_hal::gpio::p0::Parts::new(nrf51_peripherals.GPIO);
    led::initialize(gpio.p0_22, gpio.p0_24, gpio.p0_28, gpio.p0_30);
    // signal that leds have initialized
    // and that the other initialization processes are going on.
    // this also means that the processor at least booted successfully.
    Red.on();

    uart::initialize(nrf51_peripherals.UART0, &mut cortex_m_peripherals.NVIC);
    time::initialize(nrf51_peripherals.RTC0, clock_frequency);
    twi::initialize(nrf51_peripherals.TWI0, gpio.p0_04, gpio.p0_02);
    mpu::initialize();
    barometer::initialize();
    battery::initialize(nrf51_peripherals.ADC, &mut cortex_m_peripherals.NVIC);
    motor::initialize(
        nrf51_peripherals.TIMER1,
        nrf51_peripherals.TIMER2,
        &mut cortex_m_peripherals.NVIC,
        &mut nrf51_peripherals.PPI,
        &mut nrf51_peripherals.GPIOTE,
        gpio.p0_20,
    );

    // done with initialization sequence
    Red.off();
}
