use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use nrf51_hal::gpio::p0::{P0_02, P0_04};
use nrf51_hal::gpio::{Disconnected, Pin};
use nrf51_hal::twi::Pins;
use nrf51_hal::Twi;
use nrf51_pac::twi0::frequency::FREQUENCY_A;
use nrf51_pac::TWI0;

pub(crate) static TWI: Mutex<OnceCell<Twi<TWI0>>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize(twi: TWI0, scl_pin: P0_04<Disconnected>, sda_pin: P0_02<Disconnected>) {
    let scl_pin = scl_pin.into_floating_input();
    let sda_pin = sda_pin.into_floating_input();

    TWI.lock().initialize(Twi::new(
        twi,
        Pins {
            scl: Pin::from(scl_pin),
            sda: Pin::from(sda_pin),
        },
        FREQUENCY_A::K400,
    ));
}
