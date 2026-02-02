//! Módulo de abstracción para interfaces de comunicación con el dispositivo ICM20948

use crate::Icm20948Error;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::SpiDevice;

#[cfg(feature = "linux")]
use linux_embedded_hal::i2cdev::linux::LinuxI2CError;

/// Error genérico para interfaces de comunicación
#[derive(Debug, Clone)]
pub enum InterfaceError<E> {
    /// Error de comunicación I2C
    I2cError(E),
    /// Error de comunicación SPI
    SpiError(E),
    /// Parámetro inválido
    InvalidParameter,
}

#[cfg(feature = "linux")]
impl From<LinuxI2CError> for InterfaceError<LinuxI2CError> {
    fn from(error: LinuxI2CError) -> Self {
        InterfaceError::I2cError(error)
    }
}

/// Trait para abstraer la comunicación con el dispositivo ICM20948
pub trait Interface {
    /// Tipo de error que puede producir la interfaz
    type Error;

    /// Escribe un registro
    fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Lee un registro
    fn read_reg(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Cambia el modo SPI o configura I2C según necesidades específicas
    fn configure(&mut self, _param: Option<u8>) -> Result<(), Self::Error> {
        // Por defecto no hace nada, se implementa en las implementaciones específicas
        Ok(())
    }
}

/// Implementación de Interface para I2C
pub struct I2cInterface<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> I2cInterface<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Crea una nueva interfaz I2C
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Consume la interfaz y devuelve el dispositivo I2C subyacente
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C, E> Interface for I2cInterface<I2C>
where
    I2C: I2c<Error = E>,
{
    type Error = InterfaceError<E>;

    fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 17];
        buffer[0] = reg;

        if data.len() > 16 {
            return Err(InterfaceError::InvalidParameter);
        }

        buffer[1..data.len() + 1].copy_from_slice(data);

        self.i2c
            .write(self.addr, &buffer[0..data.len() + 1])
            .map_err(InterfaceError::I2cError)
    }

    fn read_reg(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c
            .write_read(self.addr, &[reg], data)
            .map_err(InterfaceError::I2cError)
    }
}

/// Implementación de Interface para SPI
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI, E> SpiInterface<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    /// Crea una nueva interfaz SPI
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Consume la interfaz y devuelve el dispositivo SPI
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI, E> Interface for SpiInterface<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    type Error = InterfaceError<E>;

    fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // Para SPI, el bit más significativo del primer byte debe ser 0 para escritura
        let write_reg = reg & 0x7F;

        let mut ops = [
            embedded_hal::spi::Operation::Write(&[write_reg]),
            embedded_hal::spi::Operation::Write(data),
        ];

        self.spi
            .transaction(&mut ops)
            .map_err(InterfaceError::SpiError)
    }

    fn read_reg(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Err(InterfaceError::InvalidParameter);
        }

        // Para SPI, el bit más significativo del primer byte debe ser 1 para lectura
        let read_reg = reg | 0x80;

        let mut ops = [
            embedded_hal::spi::Operation::Write(&[read_reg]),
            embedded_hal::spi::Operation::Read(data),
        ];

        self.spi
            .transaction(&mut ops)
            .map_err(InterfaceError::SpiError)
    }

    fn configure(&mut self, _param: Option<u8>) -> Result<(), Self::Error> {
        // NO IMPLEMENTADO
        Ok(())
    }
}

// Convertir errores de la interfaz a Icm20948Error
impl<E> From<InterfaceError<E>> for Icm20948Error
where
    Icm20948Error: From<E>,
{
    fn from(error: InterfaceError<E>) -> Self {
        match error {
            InterfaceError::I2cError(e) => Icm20948Error::from(e),
            InterfaceError::SpiError(e) => Icm20948Error::from(e),

            InterfaceError::InvalidParameter => Icm20948Error::Device,
        }
    }
}
