use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// Error for sensor operations.
pub enum Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    Write(<I2c as Write>::Error),
    WriteRead(<I2c as WriteRead>::Error),
    WrongDevice,
}

impl<I2c> core::fmt::Debug for Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), core::fmt::Error> {
        match self {
            Error::WriteRead(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Error::Write(e) => f.debug_tuple("WriteError").field(e).finish(),
            Error::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}
