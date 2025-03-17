use nrf51_pac::RADIO;
use crate::flash::FlashError;
use crate::led::Led;
use crate::time::delay_ms_assembly;

pub enum RadioError {

}
pub(crate) fn initialize (radio: RADIO) -> Result<(), FlashError> {
    // turn on the power for the RADIO peripheral
    radio.power.modify(|_radio, bit| {bit.power().enabled()});

    //
    let on = radio.power.read().power().is_enabled();
    if on == true {
        Led::Green.on();
        delay_ms_assembly(10);
        Led::Green.off();
    } else {
        Led::Red.on();
        delay_ms_assembly(10);
        Led::Red.off();
    }
    Ok(())
}