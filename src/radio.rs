use nrf51_pac::RADIO;
use crate::flash::FlashError;
use crate::led::Led;

pub enum RadioError {

}
pub(crate) fn initialize (radio: RADIO) -> Result<(), FlashError> {
    // turn on the power for the RADIO peripheral
    radio.power.modify(|_radio, bit| {bit.power().enabled()});

    //
    let on = radio.power.read().power().is_enabled();
    if on == true {
        Led::Green.on();
    } else {
        Led::Red.on();
    }
    Ok(())
}