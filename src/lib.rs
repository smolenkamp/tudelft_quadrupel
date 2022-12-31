#![no_std]
#![feature(strict_provenance)]
// #![deny(missing_docs)]
// #![deny(warnings)]
// #![deny(unused_import_braces)]
// #![deny(unused_results)]
#![deny(trivial_casts)]
#![deny(trivial_numeric_casts)]
#![deny(unused_qualifications)]
//! # tudelft quadrupel support library

extern crate alloc;
pub extern crate nrf51_hal;

pub use cortex_m_rt::entry;
pub use nb::block;

pub use cortex_m;
pub use cortex_m_rt;
pub use fixed;
pub use nrf51_pac;
pub use ringbuffer;

pub mod mutex;
pub mod once_cell;
// TODO
pub mod battery;
pub mod barometer;
pub mod initialize;
pub mod led;
// TODO
pub mod flash;
pub mod motor;
pub mod mpu;
pub mod time;
mod twi;
pub mod uart;
// TODO
mod spi;
