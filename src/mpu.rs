use crate::mpu::config::DigitalLowPassFilter;
use crate::mpu::error::Error;
use crate::mpu::sensor::Mpu6050;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::twi::TWI;
use nb::Error::WouldBlock;
use nrf51_hal::Twi;
use nrf51_pac::TWI0;
use structs::{Accel, Gyro, Quaternion};

#[allow(unused)]
mod config;
mod dmp_firmware;
mod error;
mod firmware_loader;
#[allow(unused)]
mod registers;
#[allow(unused)]
mod sensor;
pub mod structs;

const SAMPLE_RATE_DIVIDER_MPU: u8 = 0;
const SAMPLE_RATE_DIVIDER_RAW: u8 = 0;

pub struct Mpu {
    mpu: Mpu6050<Twi<TWI0>>,
    dmp_enabled: bool,
}

static MPU: Mutex<OnceCell<Mpu>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    let twi: &mut Twi<_> = &mut TWI.lock();

    let mut mpu = Mpu6050::new(twi).unwrap();
    mpu.initialize_dmp(twi).unwrap();
    mpu.set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU)
        .unwrap();
    mpu.set_digital_lowpass_filter(twi, DigitalLowPassFilter::Filter5)
        .unwrap();
    MPU.lock().initialize(Mpu {
        mpu,
        dmp_enabled: true,
    })
}

/// Is the DMP (digital motion processor) of the MPU enabled?
/// It is enabled by default.
pub fn is_dmp_enabled() -> bool {
    MPU.lock().dmp_enabled
}

/// Disable the DMP (digital motion processor) of the MPU
pub fn disable_dmp() {
    let twi: &mut Twi<_> = &mut TWI.lock();
    let mpu: &mut Mpu = &mut MPU.lock();

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_RAW)
        .unwrap();
    mpu.mpu.disable_dmp(twi).unwrap();
    mpu.mpu.disable_fifo(twi).unwrap();
    mpu.dmp_enabled = false
}

/// Enable the DMP (digital motion processor) of the MPU
pub fn enable_dmp() -> Result<(), Error<Twi<TWI0>>> {
    let twi: &mut Twi<_> = &mut TWI.lock();
    let mpu: &mut Mpu = &mut MPU.lock();

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU)?;
    mpu.mpu.enable_dmp(twi)?;
    mpu.mpu.enable_fifo(twi)?;
    mpu.dmp_enabled = true;

    Ok(())
}

/// This reads the most recent angle from the DMP, if there are any new ones available.
/// If there is no new angle available, it returns `WouldBlock`.
pub fn read_dmp_bytes() -> nb::Result<Quaternion, Error<Twi<TWI0>>> {
    let twi: &mut Twi<_> = &mut TWI.lock();
    let mpu: &mut Mpu = &mut MPU.lock();

    assert!(mpu.dmp_enabled);

    // If there isn't a full packet ready, return none
    let mut len = mpu.mpu.get_fifo_count(twi)?;
    if len < 28 {
        return Err(WouldBlock);
    }

    // Keep reading while there are more full packets
    let mut buf = [0; 28];
    while len >= 28 {
        mpu.mpu.read_fifo(twi, &mut buf)?;
        len -= 28;
    }

    // Convert the last full packet we received to a Quaternion
    Ok(Quaternion::from_bytes(&buf[..16]))
}

/// This reads the most recent acceleration and gyroscope information from the MPU.
pub fn read_raw() -> Result<(Accel, Gyro), Error<Twi<TWI0>>> {
    let twi: &mut Twi<_> = &mut TWI.lock();
    let mpu: &mut Mpu = &mut MPU.lock();

    let accel = mpu.mpu.accel(twi)?;
    let gyro = mpu.mpu.gyro(twi)?;

    Ok((accel, gyro))
}
