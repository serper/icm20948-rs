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
pub mod firmware;
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

// Importaciones necesarias para resolver errores de compilación
use crate::interface::{Interface, I2cInterface, SpiInterface};

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

/// Crea un nuevo controlador DMP para el ICM20948
pub fn new_dmp_driver<I, D, E>(
    device: Icm20948<I, D>
) -> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    Icm20948Error: From<E>
{
    device
}

/// Ejecuta un ejemplo de uso de DMP (Digital Motion Processor)
pub fn run_dmp_example<I, D, E>(
    mut device: Icm20948<I, D>
) -> Result<(), Icm20948Error>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    Icm20948Error: From<E>,
{
    // Inicializar el dispositivo
    device.initialize()?;
    println!("Dispositivo inicializado");
    
    // Configurar y activar el DMP
    println!("Configurando DMP...");
    
    // Configurar escalas para el acelerómetro y giroscopio
    device.set_accel_fullscale(types::AccelFullScale::Fs2G)?;
    device.set_gyro_fullscale(types::GyroFullScale::Fs250Dps)?;
    
    // Configurar tasas de muestreo
    device.set_accelerometer_sample_rate(controls::SampleRate::Custom(100))?; // 100Hz
    device.set_gyroscope_sample_rate(controls::SampleRate::Custom(100))?; // 100Hz

    // Configurar LPF (Low Pass Filter) para el acelerómetro y giroscopio
    device.set_accel_lpf(controls::AccelLpfSetting::Lp246_0Hz)?;
    device.set_gyro_lpf(controls::GyroLpfSetting::Lp196_6Hz)?;
    
    // Leer algunos datos
    for _ in 0..5 {
        // Leer datos de acelerómetro
        if let Ok(accel) = device.accel_read_hw_reg_data() {
            println!("Aceleración: x={:.2}, y={:.2}, z={:.2}", accel[0], accel[1], accel[2]);
        }
        
        // Leer datos de giroscopio
        if let Ok(gyro) = device.gyro_read_hw_reg_data() {
            println!("Giroscopio: x={:.2}, y={:.2}, z={:.2}", gyro[0], gyro[1], gyro[2]);
        }
        
        // Leer datos de temperatura
        if let Ok(temp) = device.read_temp_raw() {
            println!("Temperatura: {:.2}°C", temp);
        }
        
        println!("-------------------");
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    
    println!("Ejemplo DMP finalizado");
    Ok(())
}
