use nrf51_pac::RADIO;
use crate::flash::FlashError;
use crate::led::Led;
use crate::time::delay_ms_assembly;

#[derive(Debug)]
pub enum RadioError {
    NoPowerError,
}
pub(crate) fn initialize (radio: &mut RADIO) -> Result<(), RadioError> {
    // turn on the power for the RADIO peripheral
    radio.power.modify(|_radio, bit| {bit.power().enabled()});

    //
    let on = radio.power.read().power().is_enabled();
    if on == true {
        Ok(())
    } else {
        Err(RadioError::NoPowerError)
    }
}