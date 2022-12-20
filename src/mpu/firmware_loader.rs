use crate::mpu::dmp_firmware::FIRMWARE;
use crate::mpu::error::Error;
use crate::mpu::registers::Register;
use crate::mpu::sensor::Mpu6050;
use embedded_hal::blocking::i2c::{Write, WriteRead};

const BANK_SIZE: usize = 256;
const CHUNK_SIZE: usize = 16;

impl<I2c> Mpu6050<I2c>
where
    I2c: Write + WriteRead,
    <I2c as WriteRead>::Error: core::fmt::Debug,
    <I2c as Write>::Error: core::fmt::Debug,
{
    pub fn load_firmware(&mut self, i2c: &mut I2c) -> Result<(), Error<I2c>> {
        self.write_memory(i2c, &FIRMWARE)
    }

    pub fn boot_firmware(&mut self, i2c: &mut I2c) -> Result<(), Error<I2c>> {
        self.write(i2c, &[Register::PrgmStart as u8, 0x04, 0x00])
    }

    fn write_memory(&mut self, i2c: &mut I2c, data: &[u8]) -> Result<(), Error<I2c>> {
        for (bank, chunk) in data.chunks(BANK_SIZE).enumerate() {
            self.write_bank(i2c, bank as u8, chunk)?;
        }
        Ok(())
    }

    fn write_bank(&mut self, i2c: &mut I2c, bank: u8, data: &[u8]) -> Result<(), Error<I2c>> {
        self.set_bank(i2c, bank)?;

        for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
            let mut prolog_and_chunk: [u8; CHUNK_SIZE + 1] = [0; CHUNK_SIZE + 1];
            prolog_and_chunk[0] = Register::MemRw as u8;
            for (i, b) in chunk.iter().enumerate() {
                prolog_and_chunk[i + 1] = *b;
            }
            self.set_memory_start_address(i2c, (i * CHUNK_SIZE) as u8)?;
            self.write(i2c, &prolog_and_chunk)?;
        }

        Ok(())
    }

    fn set_bank(&mut self, i2c: &mut I2c, bank: u8) -> Result<(), Error<I2c>> {
        self.write_register(i2c, Register::BankSel, bank)
    }

    fn set_memory_start_address(&mut self, i2c: &mut I2c, addr: u8) -> Result<(), Error<I2c>> {
        self.write_register(i2c, Register::MemStartAddr, addr)
    }
}
