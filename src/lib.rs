#![no_std]
#![feature(strict_provenance)]
#![deny(missing_docs)]
#![deny(warnings)]
#![deny(unused_import_braces)]
#![deny(unused_results)]
#![deny(trivial_casts)]
#![deny(trivial_numeric_casts)]
#![deny(unused_qualifications)]
//! # tudelft quadrupel support library

extern crate alloc;
pub extern crate nrf51_hal;

pub use cortex_m_rt::entry;

pub use ringbuffer;
pub use cortex_m;
pub use nrf51_pac;
pub use cortex_m_rt;

pub mod once_cell;
pub mod mutex;
pub mod battery;
pub mod barometer;
pub mod led;
pub mod initialize;
pub mod flash;
pub mod time;
pub mod twi;
pub mod uart;
pub mod motor;