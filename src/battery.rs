use cortex_m::peripheral::NVIC;
use nrf51_pac::interrupt;
use nrf51_pac::Interrupt;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;

struct Adc {
    adc: nrf51_pac::ADC,
    last_result: u16,
}
static ADC_STATE: Mutex<OnceCell<Adc>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize(adc: nrf51_pac::ADC, nvic: &mut NVIC) {
    //We want to use Analog Input 4 as an input.
    //We want to use an analog input with two thirds prescaling
    adc.config.write(|w| {
        w.psel()
            .analog_input4()
            .inpsel()
            .analog_input_two_thirds_prescaling()
    });

    //We want to enable ADC now
    adc.enable.write(|w| w.enable().enabled());

    //We want to enable interrupt on ADC sample ready event, priority 3
    adc.intenset.write(|w| w.end().set_bit());
    unsafe {
        nvic.set_priority(Interrupt::ADC, 3);
    }

    ADC_STATE.lock().initialize(Adc {
        adc,
        last_result: 0,
    });

    // Safety: The initialize function is not called inside of an interrupt-free section.
    unsafe { NVIC::unmask(Interrupt::ADC); }
}



#[interrupt]
unsafe fn ADC() {
    let adc = &mut **ADC_STATE.lock();
    adc.adc.events_end.reset();
    // Battery voltage = (result*1.2*3/255*2) = RESULT*0.007058824
    adc.last_result = adc.adc.result.read().result().bits() * 7;
}

/// Voltage in 10^-2 volt
pub fn read_battery() -> u16 {
    let adc = &mut **ADC_STATE.lock();

    if !adc.adc.busy.read().busy().bit() {
        //For some reason, there is no field inside this register, so we set it to 1 manually.
        adc.adc.tasks_start.write(|w| unsafe { w.bits(1) });
    }

    adc.last_result
}
