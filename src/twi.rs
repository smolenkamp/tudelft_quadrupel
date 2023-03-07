use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use nrf51_hal::gpio::p0::{P0_02, P0_04};
use nrf51_hal::gpio::{Disconnected, Pin};
use nrf51_hal::twi::Error;
use nrf51_pac::twi0::frequency::FREQUENCY_A;
use nrf51_pac::{GPIO, TWI0};

const FREQ: FREQUENCY_A = FREQUENCY_A::K400;

pub(crate) static TWI: Mutex<OnceCell<TwiWrapper>> = Mutex::new(OnceCell::uninitialized());

pub struct TwiWrapper {
    twi: TWI0,
}

impl TwiWrapper {
    fn send_byte(&self, byte: u8) -> Result<(), Error> {
        // Clear sent event.
        self.twi.events_txdsent.write(|w| unsafe { w.bits(0) });

        // Copy data into the send buffer.
        self.twi.txd.write(|w| unsafe { w.bits(u32::from(byte)) });

        // Wait until transmission was confirmed.
        while self.twi.events_txdsent.read().bits() == 0 {
            // Bail out if we get an error instead.
            if self.twi.events_error.read().bits() != 0 {
                self.twi.events_error.write(|w| unsafe { w.bits(0) });
                return Err(Error::Transmit);
            }
        }

        // Clear sent event.
        self.twi.events_txdsent.write(|w| unsafe { w.bits(0) });

        Ok(())
    }

    fn recv_byte(&self) -> Result<u8, Error> {
        // Wait until something ended up in the buffer.
        while self.twi.events_rxdready.read().bits() == 0 {
            // Bail out if it's an error instead of data.
            if self.twi.events_error.read().bits() != 0 {
                self.twi.events_error.write(|w| unsafe { w.bits(0) });
                return Err(Error::Receive);
            }
        }

        // Read out data.
        let out = self.twi.rxd.read().bits() as u8;

        // Clear reception event.
        self.twi.events_rxdready.write(|w| unsafe { w.bits(0) });

        Ok(out)
    }

    fn send_stop(&self) -> Result<(), Error> {
        // Clear stopped event.
        self.twi.events_stopped.write(|w| unsafe { w.bits(0) });

        // Start stop condition.
        self.twi.tasks_stop.write(|w| unsafe { w.bits(1) });

        // Wait until stop was sent.
        while self.twi.events_stopped.read().bits() == 0 {
            // Bail out if we get an error instead.
            if self.twi.events_error.read().bits() != 0 {
                self.twi.events_error.write(|w| unsafe { w.bits(0) });
                return Err(Error::Transmit);
            }
        }

        Ok(())
    }
}

impl embedded_hal::blocking::i2c::Write for TwiWrapper {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        cortex_m::interrupt::free(|_| {
            // Make sure all previously used shortcuts are disabled.
            self.twi
                .shorts
                .write(|w| w.bb_stop().disabled().bb_suspend().disabled());

            // Set Slave I2C address.
            self.twi
                .address
                .write(|w| unsafe { w.address().bits(addr.into()) });

            // Start data transmission.
            self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });

            // Clock out all bytes.
            for byte in bytes {
                self.send_byte(*byte)?;
            }

            // Send stop.
            self.send_stop()?;
            Ok(())
        })
    }
}

impl embedded_hal::blocking::i2c::Read for TwiWrapper {
    type Error = Error;

    fn read(&mut self, addr: u8, bytes: &mut [u8]) -> Result<(), Error> {
        cortex_m::interrupt::free(|_| {
            // Make sure all previously used shortcuts are disabled.
            self.twi
                .shorts
                .write(|w| w.bb_stop().disabled().bb_suspend().disabled());

            // Set Slave I2C address.
            self.twi
                .address
                .write(|w| unsafe { w.address().bits(addr.into()) });

            // Read into buffer.
            if let Some((last, before)) = bytes.split_last_mut() {
                // If we want to read multiple bytes we need to use the suspend mode.
                if !before.is_empty() {
                    self.twi.shorts.write(|w| w.bb_suspend().enabled());
                } else {
                    self.twi.shorts.write(|w| w.bb_stop().enabled());
                }

                // Clear reception event.
                self.twi.events_rxdready.write(|w| unsafe { w.bits(0) });

                // Start data reception.
                self.twi.tasks_startrx.write(|w| unsafe { w.bits(1) });

                for byte in &mut before.into_iter() {
                    self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });
                    *byte = self.recv_byte()?;
                }

                self.twi.shorts.write(|w| w.bb_stop().enabled());
                self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });
                *last = self.recv_byte()?;
            } else {
                self.send_stop()?;
            }
            Ok(())
        })
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
        cortex_m::interrupt::free(|_| {
            // Make sure all previously used shortcuts are disabled.
            self.twi
                .shorts
                .write(|w| w.bb_stop().disabled().bb_suspend().disabled());

            // Set Slave I2C address.
            self.twi
                .address
                .write(|w| unsafe { w.address().bits(addr.into()) });

            // Start data transmission.
            self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });

            // Send out all bytes in the outgoing buffer.
            for byte in bytes {
                self.send_byte(*byte)?;
            }

            // Turn around to read data.
            if let Some((last, before)) = buffer.split_last_mut() {
                // If we want to read multiple bytes we need to use the suspend mode.
                if !before.is_empty() {
                    self.twi.shorts.write(|w| w.bb_suspend().enabled());
                } else {
                    self.twi.shorts.write(|w| w.bb_stop().enabled());
                }

                // Clear reception event.
                self.twi.events_rxdready.write(|w| unsafe { w.bits(0) });

                // Start data reception.
                self.twi.tasks_startrx.write(|w| unsafe { w.bits(1) });

                for byte in &mut before.into_iter() {
                    self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });
                    *byte = self.recv_byte()?;
                }

                self.twi.shorts.write(|w| w.bb_stop().enabled());
                self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });
                *last = self.recv_byte()?;
            } else {
                self.send_stop()?;
            }
            Ok(())
        })
    }
}

pub(crate) fn initialize(twi: TWI0, scl_pin: P0_04<Disconnected>, sda_pin: P0_02<Disconnected>) {
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

    // Set frequency.
    twi.frequency.write(|w| w.frequency().variant(FREQ));

    twi.enable.write(|w| w.enable().enabled());

    // Initialize oncecell
    TWI.modify(|t| t.initialize(TwiWrapper {
        twi
    }));
}
