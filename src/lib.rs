//! Biblioteca Rust para el sensor de movimiento InvenSense ICM20948
//!
//! Esta biblioteca proporciona una interfaz para controlar el sensor ICM20948,
//! un IMU de 9 ejes con giroscopio, acelerómetro y magnetómetro.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::SpiDevice;

// Importaciones internas
pub mod augmented;
pub mod base;
pub mod compass;
pub mod config;
pub mod controls;
pub mod conversion;
pub mod device;
pub mod dmp;
pub mod dmp3_firmware;
pub mod dmp_fifo;
pub mod fifo;
pub mod interface;
pub mod register;
pub mod selftest;
pub mod types;

#[cfg(feature = "rhai")]
pub mod api_rhai;
#[cfg(feature = "rhai")]
pub mod geomag;

// Re-exports públicos
pub use controls::{AccelLpfSetting, GyroLpfSetting, SampleRate};
pub use conversion::{accel_raw_to_g, gyro_raw_to_dps, temp_raw_to_celsius};
pub use device::{Icm20948, Icm20948Error};
pub use types::{AccelFullScale, GyroFullScale};

use crate::interface::{I2cInterface, SpiInterface};

/// Crea un nuevo dispositivo ICM20948 usando el bus I2C
pub fn new_i2c_device<I, D, E>(i2c: I, address: u8, delay: D) -> Icm20948<I2cInterface<I>, D>
where
    I: I2c<Error = E>,
    D: DelayNs,
{
    let interface = I2cInterface::new(i2c, address);
    Icm20948::new(interface, delay)
}

/// Crea un nuevo dispositivo ICM20948 usando un dispositivo SPI
pub fn new_spi_device<SPI, D, E>(spi: SPI, delay: D) -> Icm20948<SpiInterface<SPI>, D>
where
    SPI: SpiDevice<Error = E>,
    D: DelayNs,
{
    let interface = SpiInterface::new(spi);
    Icm20948::new(interface, delay)
}
