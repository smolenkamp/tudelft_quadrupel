use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::peripheral::NVIC;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use nrf51_hal::gpio::p0::{P0_02, P0_04};
use nrf51_hal::gpio::{Disconnected, Pin};
use nrf51_hal::twi::Error;
use nrf51_pac::twi0::frequency::FREQUENCY_A;
use nrf51_pac::{GPIO, Interrupt, TWI0};
use nrf51_pac::interrupt;

const FREQ: FREQUENCY_A = FREQUENCY_A::K400;

pub(crate) static TWI: Mutex<OnceCell<TwiWrapper>> = Mutex::new(OnceCell::uninitialized());

pub struct TwiWrapper {
    twi: TWI0,
    sent: AtomicBool,
    rec: AtomicBool,
}

impl embedded_hal::blocking::i2c::Write for TwiWrapper {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Clear sent/rec in case some event has been received in the meantime
        self.sent.store(false, Ordering::SeqCst);

        // Setup for task
        self.twi.address.write(|w| unsafe { w.address().bits(addr) });
        self.twi.shorts.write(|w| w.bb_stop().disabled().bb_suspend().disabled());

        // Write data
        self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });
        for byte in bytes {
            self.twi.txd.write(|w| w.txd().variant(*byte));
            while !self.sent.load(Ordering::SeqCst) {}
            self.sent.store(false, Ordering::SeqCst);
        }
        self.twi.tasks_stop.write(|w| unsafe { w.bits(1) });

        Ok(())
    }
}

impl embedded_hal::blocking::i2c::WriteRead for TwiWrapper {
    type Error = Error;

    fn write_read<'w>(
        &mut self,
        addr: u8,
        bytes: &'w [u8],
        buffer: &'w mut [u8],
    ) -> Result<(), Error> {
        // Clear sent/rec in case some event has been received in the meantime
        self.sent.store(false, Ordering::SeqCst);
        self.rec.store(false, Ordering::SeqCst);

        // Setup for task
        self.twi.address.write(|w| unsafe { w.address().bits(addr) });
        self.twi.shorts.write(|w| w.bb_stop().disabled().bb_suspend().disabled());
        self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });

        // Write data
        self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });
        for byte in bytes {
            self.twi.txd.write(|w| w.txd().variant(*byte));
            while !self.sent.load(Ordering::SeqCst) {}
            self.sent.store(false, Ordering::SeqCst);
        }

        // Read data
        self.twi.shorts.write(|w| w.bb_suspend().set_bit());
        self.twi.tasks_startrx.write(|w| unsafe { w.bits(1) });
        let last = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            // After the last byte, stop tasks
            if i == last {
                self.twi.shorts.write(|w| w.bb_suspend().clear_bit().bb_stop().set_bit());
            }

            while !self.rec.load(Ordering::SeqCst) {}
            self.rec.store(false, Ordering::SeqCst);
            *byte = self.twi.rxd.read().bits() as u8;
            self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });
        }

        Ok(())
    }
}

pub(crate) fn initialize(twi: TWI0, scl_pin: P0_04<Disconnected>, sda_pin: P0_02<Disconnected>, nvic: &mut NVIC) {
    let scl_pin = Pin::from(scl_pin.into_floating_input());
    let sda_pin = Pin::from(sda_pin.into_floating_input());

    // The TWIM peripheral requires the pins to be in a mode that is not
    // exposed through the GPIO API, and might it might not make sense to
    // expose it there.
    //
    // Until we've figured out what to do about this, let's just configure
    // the pins through the raw peripheral API. All of the following is
    // safe, as we own the pins now and have exclusive access to their
    // registers.
    for &pin in &[scl_pin.pin(), sda_pin.pin()] {
        unsafe { &*GPIO::ptr() }.pin_cnf[pin as usize].write(|w| {
            w.dir()
                .input()
                .input()
                .connect()
                .pull()
                .pullup()
                .drive()
                .s0d1()
                .sense()
                .disabled()
        });
    }

    // Set pins.
    twi.pselscl
        .write(|w| unsafe { w.bits(scl_pin.pin().into()) });
    twi.pselsda
        .write(|w| unsafe { w.bits(sda_pin.pin().into()) });

    // Clear interrupts
    twi.events_rxdready.reset();
    twi.events_txdsent.reset();

    // Set frequency.
    twi.frequency.write(|w| w.frequency().variant(FREQ));

    // Set which interrupts we want to receive
    twi.intenset.write(|w| w.txdsent().set_bit().rxdready().set_bit().error().set_bit());
    twi.enable.write(|w| w.enable().enabled());

    // Initialize oncecell
    TWI.modify(|t| t.initialize(TwiWrapper {
        twi,
        rec: false.into(),
        sent: false.into(),
    }));

    // Setup NVIC
    NVIC::unpend(Interrupt::SPI0_TWI0);
    // Safety: We are not using priority-based critical sections.
    unsafe {
        nvic.set_priority(Interrupt::SPI0_TWI0, 3); // Same as C template
        NVIC::unmask(Interrupt::SPI0_TWI0);
    }
}

#[interrupt]
unsafe fn SPI0_TWI0() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    // We might be accessing the hardware while the interrupted code also wants to, this is fine since we're only touching the EVENT registers which are not touched by the other code
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    if twi.twi.events_rxdready.read().bits() != 0 {
        twi.twi.events_rxdready.reset();
        twi.rec.store(true, Ordering::SeqCst);
    }
    if twi.twi.events_txdsent.read().bits() != 0 {
        twi.twi.events_txdsent.reset();
        twi.sent.store(true, Ordering::SeqCst);
    }

    // Errors are silently ignored
    if twi.twi.events_error.read().bits() != 0 {
        twi.twi.errorsrc.write(|w| w.anack().clear_bit().overrun().clear_bit()); // Clear error source
        twi.twi.events_error.reset();
    }
}