use nrf51_hal::gpio::Level;
use core::arch::asm;
use nrf51_hal::prelude::OutputPin;
use nrf51_hal::spi::{FullDuplex, MODE_0};
use nb::block;
use nrf51_hal::gpio::p0::*;
use nrf51_hal::gpio::{Disconnected, Output, PushPull};
use nrf51_hal::spi::{Frequency, Pins};
use nrf51_hal::Spi;
use nrf51_pac::SPI1;

const WRSR: u8 = 0x01;
const BYTEWRITE: u8 = 0x02;
const BYTEREAD: u8 = 0x03;
const WRDI: u8 = 0x04;
// const RDSR: u8 = 0x05;
const WREN: u8 = 0x06;
const EWSR: u8 = 0x50;
const CHIP_ERASE: u8 = 0x60;
const AAI: u8 = 0xAF;

#[derive(Debug)]
pub enum FlashError {
    SpiError(nrf51_hal::spi::Error),
    OutOfSpace,
}

impl From<void::Void> for FlashError {
    fn from(v: void::Void) -> Self {
        match v {}
    }
}

impl From<nrf51_hal::spi::Error> for FlashError {
    fn from(e: nrf51_hal::spi::Error) -> Self {
        FlashError::SpiError(e)
    }
}

pub struct SpiFlash {
    spi: Spi<SPI1>,
    _pin_wp: P0_00<Output<PushPull>>,
    _pin_hold: P0_13<Output<PushPull>>,
    pin_cs: P0_17<Output<PushPull>>,
}

impl SpiFlash {
    pub fn new(
        spi1: SPI1,
        pin_cs: P0_17<Disconnected>,
        pin_miso: P0_18<Disconnected>,
        pin_wp: P0_00<Disconnected>,
        pin_hold: P0_13<Disconnected>,
        pin_sck: P0_11<Disconnected>,
        pin_mosi: P0_09<Disconnected>,
    ) -> Result<Self, FlashError> {
        let spi = Spi::new(
            spi1,
            Pins {
                sck: Some(pin_sck.into_push_pull_output(Level::Low).degrade()),
                mosi: Some(pin_mosi.into_push_pull_output(Level::Low).degrade()),
                miso: Some(pin_miso.into_floating_input().degrade()),
            },
            Frequency::M4,
            MODE_0,
        );
        let pin_wp = pin_wp.into_push_pull_output(Level::High);
        let pin_hold = pin_hold.into_push_pull_output(Level::High);
        let pin_cs = pin_cs.into_push_pull_output(Level::High);

        let mut spi_flash = SpiFlash {
            spi,
            _pin_wp: pin_wp,
            _pin_hold: pin_hold,
            pin_cs,
        };
        spi_flash.flash_enable_wsr()?;
        spi_flash.flash_set_wrsr()?;
        spi_flash.flash_chip_erase()?;
        spi_flash.flash_write_enable()?;
        return Ok(spi_flash);
    }

    fn spi_master_tx_rx(&mut self, tx_data: &[u8], rx_data: &mut [u8]) -> Result<(), FlashError> {
        assert!(tx_data.len() != rx_data.len() || tx_data.len() == 0);

        // Enable slave
        self.pin_cs.set_low()?;

        // Write & read bytes
        block!(self.spi.send(tx_data[0]))?;
        for i in 0..tx_data.len() - 1 {
            block!(self.spi.send(tx_data[i + 1]))?;
            rx_data[i] = block!(self.spi.read())?;
        }
        *rx_data.last_mut().unwrap() = block!(self.spi.read())?;

        // Disable slave
        self.pin_cs.set_high()?;

        Ok(())
    }

    fn spi_master_tx(&mut self, tx_data: &[u8]) -> Result<(), FlashError> {
        assert_ne!(tx_data.len(), 0);

        // Enable slave
        self.pin_cs.set_low()?;

        block!(self.spi.send(tx_data[0]))?;
        for i in 0..tx_data.len() - 1 {
            block!(self.spi.send(tx_data[i + 1]))?;
            _ = block!(self.spi.read())?;
        }
        _ = block!(self.spi.read())?;

        // Disable slave
        self.pin_cs.set_high()?;

        Ok(())
    }

    // fn spi_master_rx(&mut self, rx_data: &mut [u8]) {
    //     assert_ne!(rx_data.len(), 0);
    //
    //     // Enable slave
    //     self.pin_cs.set_low().unwrap();
    //
    //     // Write & read bytes
    //     block!(self.spi.send(0)).unwrap();
    //     for i in 0..rx_data.len() - 1 {
    //         block!(self.spi.send(0)).unwrap();
    //         rx_data[i] = block!(self.spi.read()).unwrap();
    //     }
    //     *rx_data.last_mut().unwrap() = block!(self.spi.read()).unwrap();
    //
    //     // Disable slave
    //     self.pin_cs.set_high().unwrap();
    // }

    fn spi_master_tx_rx_fast_read(
        &mut self,
        tx_data: &[u8; 4],
        rx_data: &mut [u8],
    ) -> Result<(), FlashError> {
        assert_ne!(rx_data.len(), 0);

        // Enable slave
        self.pin_cs.set_low()?;

        for byte in tx_data {
            _ = block!(self.spi.send(*byte))?;
            _ = block!(self.spi.read())?;
        }

        for byte in rx_data {
            _ = block!(self.spi.send(0))?;
            *byte = block!(self.spi.read())?;
        }

        // Disable slave
        self.pin_cs.set_high()?;

        Ok(())
    }

    fn spi_master_tx_rx_fast_write(
        &mut self,
        tx_data: &[u8; 4],
        bytes: &[u8],
    ) -> Result<(), FlashError> {
        assert_ne!(bytes.len(), 0);

        let mut bytes_written: u32 = 0;
        let address: u32 =
            (tx_data[3] as u32) + ((tx_data[2] as u32) << 8) + ((tx_data[1] as u32) << 16);

        // Enable slave
        self.pin_cs.set_low()?;

        for byte in tx_data {
            block!(self.spi.send(*byte))?;
            _ = block!(self.spi.read())?;
        }

        // Send first byte
        block!(self.spi.send(bytes[0]))?;
        _ = block!(self.spi.read())?;

        // Disable slave
        self.pin_cs.set_high()?;

        for i in 1..bytes.len() {
            nrf_delay_us(15);

            // Enable slave
            self.pin_cs.set_low()?;
            block!(self.spi.send(AAI))?;
            _ = block!(self.spi.read())?;

            block!(self.spi.send(bytes[i]))?;
            _ = block!(self.spi.read())?;

            bytes_written += 1;

            // Disable slave
            self.pin_cs.set_high()?;

            if address + bytes_written >= 0x1FFFF && i < bytes.len() - 1 {
                return Err(FlashError::OutOfSpace);
            }
        }

        nrf_delay_us(20);

        // Enable slave
        self.pin_cs.set_low()?;

        //Send WRDI
        block!(self.spi.send(WRDI))?;
        _ = block!(self.spi.read())?;

        // Disable slave
        self.pin_cs.set_high()?;

        Ok(())
    }

    /// Write-Enable(WREN).
    fn flash_write_enable(&mut self) -> Result<(), FlashError> {
        self.spi_master_tx(&[WREN])
    }

    // /// Write-Disable(WRDI).
    // fn flash_write_disable(&mut self) {
    //     self.spi_master_tx(&[WRDI]);
    // }

    /// Clears all memory locations by setting value to 0xFF.
    pub fn flash_chip_erase(&mut self) -> Result<(), FlashError> {
        self.flash_write_enable()?;
        self.spi_master_tx(&[CHIP_ERASE])?;
        nrf_delay_ms(100);
        Ok(())
    }

    // /// Read Read-Status-Register (RDSR).
    // fn flash_read_status(&mut self) -> u8 {
    //     let mut rx_data = [0u8; 2];
    //     self.spi_master_tx_rx(&[RDSR, 0x00], &mut rx_data);
    //     return rx_data[1];
    // }

    /// Enable-Write-Status-Register (EWSR). This function must be followed by flash_enable_WSR().
    fn flash_enable_wsr(&mut self) -> Result<(), FlashError> {
        self.spi_master_tx(&[EWSR])
    }

    /// Sets Write-Status-Register (WRSR) to 0x00 to enable memory write.
    fn flash_set_wrsr(&mut self) -> Result<(), FlashError> {
        self.spi_master_tx(&[WRSR, 0x00])
    }

    /// Writes one byte data to specified address.
    ///
    /// @note Make sure that the memory location is cleared before writing data. If data is already present
    ///       in the memory location (given address), new data cannot be written to that memory location unless
    ///   	 flash_chip_erase() function is called.
    ///
    /// @param address any address between 0x000000 to 0x01FFFF where the data should be stored.
    /// @param data one byte data to be stored.
    pub fn flash_write_byte(&mut self, address: u32, byte: u8) -> Result<(), FlashError> {
        self.flash_write_enable()?;
        self.spi_master_tx(&[
            BYTEWRITE,
            address.to_ne_bytes()[2],
            address.to_ne_bytes()[1],
            address.to_ne_bytes()[0],
            byte,
        ])?;
        nrf_delay_us(20);
        Ok(())
    }

    /// Writes multi-byte data into memory starting from specified address. Each memory location (address)
    /// holds one byte of data.
    ///
    /// @note Make sure that the memory location is cleared before writing data. If data is already present
    ///       in the memory location (given address), new data cannot be written to that memory location unless
    ///   	 flash_chip_erase() function is called.
    ///
    /// @param address starting address (between 0x000000 to 0x01FFFF) from which the data should be stored.
    /// @param byte pointer to uint8_t type array containing data.
    pub fn flash_write_bytes(&mut self, address: u32, bytes: &[u8]) -> Result<(), FlashError> {
        self.flash_write_enable()?;
        self.spi_master_tx_rx_fast_write(
            &[
                AAI,
                address.to_ne_bytes()[2],
                address.to_ne_bytes()[1],
                address.to_ne_bytes()[0],
            ],
            bytes,
        )?;
        Ok(())
    }

    /// Reads one byte data from specified address.
    ///
    /// @param address any address between 0x000000 to 0x01FFFF from where the data should be read.
    ///                The address is incremented automatically and once the data is written to last accessible
    ///                address - 0x01FFFF, the function returns immediately with failure if there is pending data to write.
    pub fn flash_read_byte(&mut self, address: u32) -> Result<u8, FlashError> {
        let mut rx_data = [0; 5];
        self.spi_master_tx_rx(
            &[
                BYTEREAD,
                address.to_ne_bytes()[2],
                address.to_ne_bytes()[1],
                address.to_ne_bytes()[0],
                0x00,
            ],
            &mut rx_data,
        )?;
        Ok(rx_data[4])
    }

    ///Reads multi-byte data starting from specified address.
    ///
    ///@param address starting address (between 0x000000 to 0x01FFFF) from which the data is read.
    ///               The address is incremented automatically and once the data from address 0x01FFFF
    ///               is read, the next location will be 0x000000.
    ///@param buffer pointer to uint8_t type array where data is stored.
    pub fn flash_read_bytes(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FlashError> {
        self.spi_master_tx_rx_fast_read(
            &[
                BYTEREAD,
                address.to_ne_bytes()[2],
                address.to_ne_bytes()[1],
                address.to_ne_bytes()[0],
            ],
            buffer,
        )?;
        Ok(())
    }
}

#[allow(unused_assignments)]
fn nrf_delay_us(mut number_of_us: u32) {
    unsafe {
        asm!(
            "1:",
            "subs {}, #1",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "nop",
            "bne 1b",
            inout(reg) number_of_us,
            options(nomem, nostack)
        )
    }
}

fn nrf_delay_ms(number_of_ms: u32) {
    for _ in 0..number_of_ms {
        nrf_delay_us(999);
    }
}
