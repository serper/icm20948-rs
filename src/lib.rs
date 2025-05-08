//! Biblioteca Rust para el sensor de movimiento InvenSense ICM20948
//! 
//! Esta biblioteca proporciona una interfaz para controlar el sensor ICM20948,
//! un IMU de 9 ejes con giroscopio, acelerómetro y magnetómetro.

use embedded_hal::blocking::delay::DelayMs;

// Importaciones internas
pub mod base;
pub mod config;
pub mod controls;
pub mod device;
pub mod dmp;
pub mod dmp3_firmware;
pub mod dmp_fifo;
pub mod fifo;
pub mod interface;
pub mod register;
pub mod selftest;
pub mod types;
pub mod compass;
pub mod augmented;
pub mod conversion;

// Re-exports públicos
pub use device::{Icm20948, Icm20948Error};
pub use types::{GyroFullScale, AccelFullScale};
pub use controls::{AccelLpfSetting, GyroLpfSetting, SampleRate};
pub use conversion::{accel_raw_to_g, gyro_raw_to_dps, temp_raw_to_celsius};

use crate::interface::{I2cInterface, SpiInterface};

/// Crea un nuevo dispositivo ICM20948 usando el bus I2C
pub fn new_i2c_device<I2C, D, E>(
    i2c: I2C, 
    address: u8, 
    delay: D
) -> Icm20948<I2cInterface<I2C>, D> 
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E> + embedded_hal::blocking::i2c::WriteRead<Error = E>,
    D: DelayMs<u32>,
    Icm20948Error: From<interface::InterfaceError<E>>
{
    let interface = I2cInterface::new(i2c, address);
    Icm20948::new(interface, delay)
}

/// Crea un nuevo dispositivo ICM20948 usando el bus SPI
pub fn new_spi_device<SPI, CS, D, E>(
    spi: SPI,
    cs: CS,
    delay: D
) -> Icm20948<SpiInterface<SPI, CS>, D>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = E> + embedded_hal::blocking::spi::Write<u8, Error = E>,
    CS: embedded_hal::digital::v2::OutputPin,
    D: DelayMs<u32>,
    Icm20948Error: From<interface::InterfaceError<E>>
{
    let interface = SpiInterface::new(spi, cs);
    Icm20948::new(interface, delay)
}
