use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use nrf51_hal::gpio::p0::{P0_02, P0_04};
use nrf51_hal::gpio::{Disconnected, Pin};
use nrf51_hal::twi::{Error, Pins};
use nrf51_hal::Twi;
use nrf51_pac::twi0::frequency::FREQUENCY_A;
use nrf51_pac::TWI0;

pub(crate) static TWI: Mutex<OnceCell<TwiWrapper>> = Mutex::new(OnceCell::uninitialized());

pub struct TwiWrapper {
    pub(crate) twi: Twi<TWI0>,
}

impl embedded_hal::blocking::i2c::Write for TwiWrapper {
    type Error = Error;

    fn write<'w>(&mut self, addr: u8, bytes: &'w [u8]) -> Result<(), Error> {
        cortex_m::interrupt::free(|_| self.twi.write(addr, bytes))
    }
}

impl embedded_hal::blocking::i2c::Read for TwiWrapper {
    type Error = Error;

    fn read<'w>(&mut self, addr: u8, bytes: &'w mut [u8]) -> Result<(), Error> {
        cortex_m::interrupt::free(|_| self.twi.read(addr, bytes))
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
        cortex_m::interrupt::free(|_| self.twi.write_then_read(addr, bytes, buffer))
    }
}

pub(crate) fn initialize(twi: TWI0, scl_pin: P0_04<Disconnected>, sda_pin: P0_02<Disconnected>) {
    let scl_pin = scl_pin.into_floating_input();
    let sda_pin = sda_pin.into_floating_input();

    TWI.modify(|t| {
        t.initialize(TwiWrapper {
            twi: Twi::new(
                twi,
                Pins {
                    scl: Pin::from(scl_pin),
                    sda: Pin::from(sda_pin),
                },
                FREQUENCY_A::K400,
            ),
        })
    });
}
