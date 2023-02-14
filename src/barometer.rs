use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::twi::TWI;
use core::time::Duration;
use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
use nrf51_hal::Twi;
use crate::time::Instant;

const MS5611_ADDR: u8 = 0b01110111;
const REG_READ: u8 = 0x0;
const REG_D1: u8 = 0x40;
const REG_D2: u8 = 0x50;
const REG_PROM: u8 = 0xA0;

#[allow(dead_code)]
enum OverSamplingRatio {
    Opt256,
    Opt512,
    Opt1024,
    Opt2048,
    Opt4096,
}

impl OverSamplingRatio {
    fn get_delay(&self) -> Duration {
        Duration::from_micros(match *self {
            OverSamplingRatio::Opt256 => 1000,
            OverSamplingRatio::Opt512 => 2000,
            OverSamplingRatio::Opt1024 => 3000,
            OverSamplingRatio::Opt2048 => 5000,
            OverSamplingRatio::Opt4096 => 10000,
        })
    }

    fn addr_modifier(&self) -> u8 {
        match *self {
            OverSamplingRatio::Opt256 => 0,
            OverSamplingRatio::Opt512 => 2,
            OverSamplingRatio::Opt1024 => 4,
            OverSamplingRatio::Opt2048 => 6,
            OverSamplingRatio::Opt4096 => 8,
        }
    }
}

enum Ms5611LoopState {
    Reset,
    ReadD1 { start_time: Instant },
    ReadD2 { start_time: Instant, d1: u32 },
}

struct Ms5611 {
    /// We store the values C1-C6 from the memory of the MS5611
    /// We need to use them for later calculations
    /// From datasheet, C1.
    pressure_sensitivity: u16,
    /// From datasheet, C2.
    pressure_offset: u16,
    /// From datasheet, C3.
    temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    temp_ref: u16,

    /// What should the oversampling ratio of the chip be?
    over_sampling_ratio: OverSamplingRatio,

    /// State of the QMs5611 chip
    loop_state: Ms5611LoopState,

    /// Pressure in 10^-5 bar
    most_recent_pressure: u32,
}

static BAROMETER: Mutex<OnceCell<Ms5611>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    let twi: &mut Twi<_> = &mut TWI.lock();

    let mut prom = [0; 8];
    let mut data = [0u8; 2];
    for c in 0..8 {
        twi.write_read(MS5611_ADDR, &[REG_PROM + 2 * c], &mut data)
            .unwrap();
        prom[c as usize] = u16::from_be_bytes(data);
    }

    BAROMETER.lock().initialize(Ms5611 {
        pressure_sensitivity: prom[1],
        pressure_offset: prom[2],
        temp_coef_pressure_sensitivity: prom[3],
        temp_coef_pressure_offset: prom[4],
        temp_ref: prom[5],
        over_sampling_ratio: OverSamplingRatio::Opt4096,
        loop_state: Ms5611LoopState::Reset,
        most_recent_pressure: 0,
    })
}

fn update() {
    let baro: &mut Ms5611 = &mut BAROMETER.lock();
    let twi: &mut Twi<_> = &mut TWI.lock();
    let now = Instant::now();

    match baro.loop_state {
        Ms5611LoopState::Reset => {
            //We let the chip know we want to read D1.
            twi.write(
                MS5611_ADDR,
                &[REG_D1 + baro.over_sampling_ratio.addr_modifier()],
            )
            .unwrap();

            //Then set loop state for next iteration
            baro.loop_state = Ms5611LoopState::ReadD1 {
                start_time: Instant::now(),
            };
        }
        Ms5611LoopState::ReadD1 { start_time } => {
            //If the chip has not had enough time to process, return
            if now - start_time < baro.over_sampling_ratio.get_delay() {
                return;
            }

            //Read D1
            let mut buf = [0u8; 4];
            twi.write_read(MS5611_ADDR, &[REG_READ], &mut buf[1..4])
                .unwrap();
            let d1 = u32::from_be_bytes(buf);

            //We let the chip know we want to read D2.
            twi.write(
                MS5611_ADDR,
                &[REG_D2 + baro.over_sampling_ratio.addr_modifier()],
            )
            .unwrap();

            //Then set loop state for next iteration
            baro.loop_state = Ms5611LoopState::ReadD2 {
                start_time: now,
                d1,
            };
        }
        Ms5611LoopState::ReadD2 { start_time, d1 } => {
            //If the chip has not had enough time to process, return
            if now - start_time < baro.over_sampling_ratio.get_delay() {
                return;
            }

            //Read D2
            let mut buf = [0u8; 4];
            twi.write_read(MS5611_ADDR, &[REG_READ], &mut buf[1..4])
                .unwrap();
            let d1 = d1 as u64;
            let d2 = u32::from_be_bytes(buf) as u64;

            //Use D1 and D2 to find the new pressure and temperature
            //Calculated using the ms5611 reference manual
            let dt = d2 - ((baro.temp_ref as u64) << 8);
            let offset: u64 = ((baro.pressure_offset as u64) << 16)
                + ((dt * (baro.temp_coef_pressure_offset as u64)) >> 7);
            let sens: u64 = ((baro.pressure_sensitivity as u64) << 15)
                + ((dt * (baro.temp_coef_pressure_sensitivity as u64)) >> 8);
            baro.most_recent_pressure = ((((d1 * sens) >> 21) - offset) >> 15) as u32;

            //Then set loop state for next iteration, and we can do the next iteration immediately
            baro.loop_state = Ms5611LoopState::Reset;
            update();
        }
    }
}

/// Returns pressure in 10^-5 bar.
/// This function will never block, instead it will return an old value if no new value is available.
pub fn read_pressure() -> u32 {
    update();
    BAROMETER.lock().most_recent_pressure
}
