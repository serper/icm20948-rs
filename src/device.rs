use crate::interface::Interface;
use crate::types::{self, data_defs, AccelFullScale, GyroFullScale};

// Actualizar importación de registros
use crate::base::{SystemTimeSource, TimeSource};
use crate::compass::CompassConfig;
use crate::controls::AccelData;
use crate::register::{
    dmp, registers::bank0, registers::bank1, registers::bank2, registers::bank3,
    registers::RegisterBank,
};
use embedded_hal::delay::DelayNs;

// Import bits from types
use crate::types::bits;

// Import gravity from types
use crate::types::gravity;

// Import conversion functions
use crate::conversion::{accel_raw_to_g, gyro_raw_to_dps, temp_raw_to_celsius};

// Define the device structure and enums
pub struct Icm20948<I, D> {
    pub(crate) interface: I,
    pub(crate) base_state: BaseState,
    pub(crate) delay: D,
}

#[derive(Debug, Clone)]
pub enum Icm20948Error {
    InterfaceError,
    InvalidParameter,
    FirmwareLoadError,
    FirmwareVerificationFailed,
    WhoAmIError,
    InvalidOperation,
    Device,
    DeviceNotFound,
    Timeout,
    Overflow,
    Other,
    // Add other error types as needed
}

impl Icm20948Error {
    pub fn from_error<E>(_error: E) -> Self {
        Icm20948Error::InterfaceError
    }
}

// Implementación para manejar errores de linux_embedded_hal
#[cfg(feature = "linux")]
impl From<linux_embedded_hal::i2cdev::linux::LinuxI2CError> for Icm20948Error {
    fn from(_error: linux_embedded_hal::i2cdev::linux::LinuxI2CError) -> Self {
        Icm20948Error::Device
    }
}

#[cfg(feature = "linux")]
impl From<linux_embedded_hal::spidev::SpidevTransfer<'_, '_>> for Icm20948Error {
    fn from(_error: linux_embedded_hal::spidev::SpidevTransfer<'_, '_>) -> Self {
        Icm20948Error::Device
    }
}

/// Estado base del dispositivo ICM20948
#[derive(Debug, Clone)]
pub struct BaseState {
    pub wake_state: u8,
    pub firmware_loaded: bool,
    pub compass_sens: [i16; 3],
    pub compass_asa: [u8; 3],
    /// Mounting matrix for accel/gyro (row-major, signed chars)
    pub mounting_matrix: [i8; 9],
    /// Soft-iron matrix for compass (row-major, Q30)
    pub soft_iron_matrix: [i32; 9],
    /// Scale/Skew matrix for accel/gyro (row-major, floats, default Identity)
    pub accel_gyro_scale_matrix: [f32; 9],
    pub temperature_c: f32,
    pub compass_config: Option<CompassConfig>,
    pub lp_en_support: u8,
    pub chip_lp_ln_mode: u8,
    pub accel_half_res: bool,
    pub pwr_mgmt_1: u8,
    pub pwr_mgmt_2: u8,
    pub last_bank_selected: u8,
    pub last_mems_bank_selected: u8,
    pub gyro_fullscale: GyroFullScale,
    pub accel_fullscale: AccelFullScale,
    pub accel_divider: u16,
    pub gyro_divider: u16,
    pub secondary_divider: u8,
    pub order: u8,
    pub dmp_enabled: bool,
    pub batch_mode: bool,
    pub enabled_sensors_0: u32,
    pub enabled_sensors_1: u32,
}

impl Default for BaseState {
    fn default() -> Self {
        Self {
            wake_state: 0,
            firmware_loaded: false,
            compass_sens: [0, 0, 0],
            compass_asa: [0, 0, 0],
            mounting_matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1],
            soft_iron_matrix: [1i32 << 30, 0, 0, 0, 1i32 << 30, 0, 0, 0, 1i32 << 30],
            temperature_c: 0.0,
            accel_gyro_scale_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            compass_config: None,
            lp_en_support: 0,
            chip_lp_ln_mode: 0,
            accel_half_res: false,
            pwr_mgmt_1: 0,
            pwr_mgmt_2: 0,
            last_bank_selected: 0xFF,
            last_mems_bank_selected: 0xFF,
            gyro_fullscale: GyroFullScale::default(),
            accel_fullscale: AccelFullScale::default(),
            accel_divider: 0,
            gyro_divider: 0,
            secondary_divider: 0,
            order: 0,
            dmp_enabled: false,
            batch_mode: false,
            enabled_sensors_0: 0,
            enabled_sensors_1: 0,
        }
    }
}

// Constantes para el tipo de interfaz serial
pub const SERIAL_INTERFACE_I2C: u8 = 0;
pub const SERIAL_INTERFACE_SPI: u8 = 1;

// Constantes para el estado de energía del chip
pub const CHIP_AWAKE: u8 = 0x01;
pub const CHIP_LP_ENABLE: u8 = 0x02;

// Máscaras para sensores
pub const GYRO_AVAILABLE: u8 = 0x1;
pub const ACCEL_AVAILABLE: u8 = 0x2;
pub const SECONDARY_COMPASS_AVAILABLE: u8 = 0x8;
pub const INV_NEEDS_ACCEL_MASK: u32 = 0xE29E8E0A;
pub const INV_NEEDS_GYRO_MASK: u32 = 0xE6018E18;
pub const INV_NEEDS_COMPASS_MASK: u32 = 0x8310480C | 0x800; // Added bit 11 for RotationVector
pub const INV_NEEDS_ACCEL_MASK1: u32 = 0x000006A8;
pub const INV_NEEDS_GYRO_MASK1: u32 = 0x00000818;
pub const INV_NEEDS_COMPASS_MASK1: u32 = 0x00000084;
pub const INV_NEEDS_PRESSURE: u32 = 0x10000040;

// Modos de operación
pub const CHIP_LOW_NOISE_ICM20948: u8 = 0;
pub const CHIP_LOW_POWER_ICM20948: u8 = 1;
pub const CHIP_LOW_POWER_WOI2C_ICM20948: u8 = 2;

// Constante para divider de FIFO
pub const FIFO_DIVIDER: u8 = 19; // 1125Hz/20 = 56.25Hz

impl<I: Clone, D: Clone> Clone for Icm20948<I, D> {
    fn clone(&self) -> Self {
        Self {
            interface: self.interface.clone(),
            base_state: self.base_state.clone(),
            delay: self.delay.clone(),
        }
    }
}

impl<I, D, E> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayNs,
{
    /// Create a new instance of Icm20948
    pub fn new(interface: I, delay: D) -> Self {
        Self {
            interface,
            base_state: BaseState::default(),
            delay,
        }
    }

    pub fn read_reg<B: RegisterBank>(&mut self, reg: u8) -> Result<u8, Icm20948Error> {
        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Leer el registro
        let mut data = [0u8];
        self.interface
            .read_reg(reg, &mut data)
            .map_err(|e: E| Icm20948Error::from_error(e))?;

        Ok(data[0])
    }

    /// Método genérico para leer múltiples registros de un banco específico
    pub fn read_regs<B: RegisterBank>(
        &mut self,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Icm20948Error> {
        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Leer los registros
        self.interface
            .read_reg(reg, data)
            .map_err(|e| Icm20948Error::from_error(e))
    }

    /// Método genérico para escribir en un registro de un banco específico
    pub fn write_reg<B: RegisterBank>(&mut self, reg: u8, value: u8) -> Result<(), Icm20948Error> {
        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Escribir en el registro
        self.interface
            .write_reg(reg, &[value])
            .map_err(|e| Icm20948Error::from_error(e))
    }

    /// Método genérico para escribir en múltiples registros de un banco específico
    pub fn write_regs<B: RegisterBank>(
        &mut self,
        reg: u8,
        values: &[u8],
    ) -> Result<(), Icm20948Error> {
        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Escribir en los registros
        self.interface
            .write_reg(reg, values)
            .map_err(|e| Icm20948Error::from_error(e))
    }

    /// Método genérico para modificar bits específicos de un registro en un banco específico
    pub fn modify_reg<B: RegisterBank, F>(&mut self, reg: u8, f: F) -> Result<(), Icm20948Error>
    where
        F: FnOnce(u8) -> u8,
    {
        // Leer el valor actual
        let value = self.read_reg::<B>(reg)?;

        // Aplicar la modificación
        let new_value = f(value);

        // Escribir el nuevo valor
        self.write_reg::<B>(reg, new_value)
    }

    pub fn read_mems_reg<B: RegisterBank>(&mut self, reg: u8) -> Result<u8, Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Leer el registro
        let mut data = [0u8];
        self.interface
            .read_reg(reg, &mut data)
            .map_err(|e: E| Icm20948Error::from_error(e))?;

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 1)?;
        }

        Ok(data[0])
    }

    /// Método genérico para leer múltiples registros de un banco específico
    pub fn read_mems_regs<B: RegisterBank>(
        &mut self,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Leer los registros
        let result = self
            .interface
            .read_reg(reg, data)
            .map_err(|e| Icm20948Error::from_error(e));

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            let restore_result = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
            if result.is_ok() {
                restore_result?;
            }
        }

        result
    }

    /// Método genérico para escribir en un registro de un banco específico
    pub fn write_mems_reg<B: RegisterBank>(
        &mut self,
        reg: u8,
        value: u8,
    ) -> Result<(), Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Escribir en el registro
        let result = self
            .interface
            .write_reg(reg, &[value])
            .map_err(|e| Icm20948Error::from_error(e));

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            let restore_result = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
            if result.is_ok() {
                restore_result?;
            }
        }

        result
    }

    /// Método genérico para escribir en múltiples registros de un banco específico
    pub fn write_mems_regs<B: RegisterBank>(
        &mut self,
        reg: u8,
        values: &[u8],
    ) -> Result<(), Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        // Seleccionar el banco correcto
        self.set_bank(B::BANK)?;

        // Escribir en los registros
        let result = self
            .interface
            .write_reg(reg, values)
            .map_err(|e| Icm20948Error::from_error(e));

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            let restore_result = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
            if result.is_ok() {
                restore_result?;
            }
        }

        result
    }

    /// Método genérico para modificar bits específicos de un registro en un banco específico
    pub fn modify_mems_reg<B: RegisterBank, F>(
        &mut self,
        reg: u8,
        f: F,
    ) -> Result<(), Icm20948Error>
    where
        F: FnOnce(u8) -> u8,
    {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        // Leer el valor actual
        let value = self.read_reg::<B>(reg)?;

        // Aplicar la modificación
        let new_value = f(value);

        // Escribir el nuevo valor
        let result = self.write_reg::<B>(reg, new_value);

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            let restore_result = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
            if result.is_ok() {
                restore_result?;
            }
        }

        result
    }

    /// Método para comprobar si el registro necesita poner el chip en modo de bajo consumo
    pub fn check_chip_lp_mode(&mut self, reg: u8) -> Result<bool, Icm20948Error> {
        match reg {
            bank0::LP_CONFIG
            | bank0::PWR_MGMT_1
            | bank0::PWR_MGMT_2
            | bank0::INT_PIN_CFG
            | bank0::INT_ENABLE
            | bank0::FIFO_COUNTH
            | bank0::FIFO_COUNTL
            | bank0::FIFO_R_W => Ok(self.is_batch_mode()),
            bank0::FIFO_CFG
            | bank0::MEM_BANK_SEL
            | bank0::REG_BANK_SEL
            | bank0::INT_STATUS
            | bank0::DMP_INT_STATUS => Ok(false),
            _ => Ok(true),
        }
    }

    /// Método para comprobar si está activado el batch mode leyendo el registro de control de salida 2
    pub fn get_batch_mode_status(&mut self) -> Result<bool, Icm20948Error> {
        let mut reg = [0u8; 2];
        self.read_mems(dmp::data_output_control::DATA_OUT_CTL2, &mut reg)
            .map_err(|_| Icm20948Error::InterfaceError)?;
        Ok((u16::from_be_bytes(reg) & dmp::output_mask2::BATCH_MODE_ENABLE) != 0)
    }

    /// Método para comrpobar si está activado el batch mode sin leer el registro de control de salida 2
    pub fn is_batch_mode(&self) -> bool {
        self.base_state.batch_mode
    }

    /// Método para leer registros con préstamo temporal del I2C
    pub fn read_regs_raw<'a>(
        &mut self,
        reg: u8,
        data: &'a mut [u8],
    ) -> Result<&'a mut [u8], Icm20948Error> {
        self.interface
            .read_reg(reg, data)
            .map_err(|_| Icm20948Error::InterfaceError)?;
        Ok(data)
    }

    /// Método para escribir registros con préstamo temporal del I2C
    pub fn write_regs_raw<'a>(&mut self, reg: u8, data: &'a [u8]) -> Result<(), Icm20948Error> {
        self.interface
            .write_reg(reg, &data)
            .map_err(|_| Icm20948Error::InterfaceError)
    }

    /// Leer un registro
    pub fn read_reg_raw(&mut self, reg: u8) -> Result<u8, Icm20948Error> {
        let mut buf = [0u8];
        self.interface
            .read_reg(reg, &mut buf)
            .map_err(|_| Icm20948Error::InterfaceError)?;

        Ok(buf[0])
    }

    /// Escribir a un registro
    pub fn write_reg_raw(&mut self, reg: u8, value: u8) -> Result<(), Icm20948Error> {
        self.interface
            .write_reg(reg, &[value])
            .map_err(|_| Icm20948Error::InterfaceError)?;

        Ok(())
    }

    /// Set bank for register access
    pub(crate) fn set_bank(&mut self, bank: u8) -> Result<(), Icm20948Error> {
        if bank > 3 {
            return Err(Icm20948Error::InvalidParameter);
        }

        // Force Write - No Read-Modify-Write to avoid potential garbage
        // Bits 3:0 and 7:6 are reserved, so writing 0 is safe/expected
        let val = (bank & 0x03) << 4;
        self.write_reg_raw(bank0::REG_BANK_SEL, val)?;
        self.base_state.last_bank_selected = bank;

        // Verify that the bank was actually selected
        let verify = self.read_reg_raw(bank0::REG_BANK_SEL)?;
        if (verify & 0x30) != val {
            // Verification failed - The chip is not in the correct bank
            return Err(Icm20948Error::InterfaceError);
        }

        Ok(())
    }

    /// Write to DMP memory
    pub fn write_mems(&mut self, mem_addr: u16, data: &[u8]) -> Result<(), Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        let mut bytes_written = 0;
        let size = data.len();

        while bytes_written < size {
            // Set the starting read or write address
            let bank_selected = ((mem_addr + bytes_written as u16) >> 8) as u8;
            if bank_selected != self.base_state.last_mems_bank_selected {
                self.write_reg::<bank0::Bank>(bank0::MEM_BANK_SEL, bank_selected)?;
                self.base_state.last_mems_bank_selected = bank_selected;
            }
            let addr = ((mem_addr + bytes_written as u16) & 0xff) as u8;
            self.write_reg::<bank0::Bank>(bank0::MEM_START_ADDR, addr)?;

            let this_len = std::cmp::min(types::INV_MAX_SERIAL_WRITE, size - bytes_written);

            // Write data
            let buffer = &data[bytes_written..bytes_written + this_len];
            if let Err(e) = self.write_regs::<bank0::Bank>(bank0::MEM_R_W, &buffer[..]) {
                // Restaurar el estado de LP_ENABLE si es necesario antes de retornar error
                if (power_state & CHIP_LP_ENABLE) != 0 {
                    let _ = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
                }
                return Err(e);
            }

            bytes_written += this_len;
        }

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 1)?;
        }

        Ok(())
    }

    /// Read from DMP memory
    pub fn read_mems(&mut self, mem_addr: u16, data: &mut [u8]) -> Result<(), Icm20948Error> {
        // Salvamos el estado de power
        let power_state = self.base_state.wake_state;

        // Despertar el chip si es necesario
        if (power_state & CHIP_AWAKE) == 0 {
            self.set_chip_power_state(CHIP_AWAKE, 1)?;
        }

        // Desactivar CHIP_LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 0)?;
        }

        let mut bytes_read = 0;
        let size = data.len();

        while bytes_read < size {
            // Set the starting read or write address
            let bank_selected = ((mem_addr + bytes_read as u16) >> 8) as u8;
            if bank_selected != self.base_state.last_mems_bank_selected {
                self.write_reg::<bank0::Bank>(bank0::MEM_BANK_SEL, bank_selected)?;
                self.base_state.last_mems_bank_selected = bank_selected;
            }
            let addr = ((mem_addr + bytes_read as u16) & 0xff) as u8;
            self.write_reg::<bank0::Bank>(bank0::MEM_START_ADDR, addr)?;

            let this_len = std::cmp::min(types::INV_MAX_SERIAL_WRITE, size - bytes_read);

            // Read data
            if let Err(e) = self.read_regs::<bank0::Bank>(
                bank0::MEM_R_W,
                &mut data[bytes_read..bytes_read + this_len],
            ) {
                // Restaurar el estado de LP_ENABLE si es necesario antes de retornar error
                if (power_state & CHIP_LP_ENABLE) != 0 {
                    let _ = self.set_chip_power_state(CHIP_LP_ENABLE, 1);
                }
                return Err(e);
            }

            bytes_read += this_len;
        }

        // Restaurar el estado de LP_ENABLE si es necesario
        if (power_state & CHIP_LP_ENABLE) != 0 {
            self.set_chip_power_state(CHIP_LP_ENABLE, 1)?;
        }

        Ok(())
    }

    /// Prevent low power mode from being enabled
    pub fn prevent_lpen_control(&mut self) {
        self.base_state.lp_en_support = 0;
    }

    /// Allow low power mode to be enabled
    pub fn allow_lpen_control(&mut self) {
        self.base_state.lp_en_support = 1;
        self.set_chip_power_state(CHIP_LP_ENABLE, 1).unwrap_or(());
    }

    /// Get low power enable control state
    pub fn get_lpen_control(&self) -> bool {
        self.base_state.lp_en_support != 0
    }

    /// Set chip power state
    pub fn set_chip_power_state(&mut self, func: u8, on_off: u8) -> Result<(), Icm20948Error> {
        match func {
            CHIP_AWAKE => {
                if (self.base_state.wake_state & CHIP_AWAKE) == 0 {
                    self.base_state.pwr_mgmt_1 &= !bits::SLEEP;
                    let result = self
                        .write_reg::<bank0::Bank>(bank0::PWR_MGMT_1, self.base_state.pwr_mgmt_1);
                    self.base_state.wake_state |= CHIP_AWAKE;
                    // After writing the bit wait 100 Micro Seconds
                    self.delay.delay_us(100); // Increased to 1ms for stability
                    return result;
                }
            }
            CHIP_LP_ENABLE => {
                if self.base_state.lp_en_support == 1 {
                    if on_off != 0 {
                        // lp_en ON
                        if self.base_state.lp_en_support != 0
                            && (self.base_state.wake_state & CHIP_LP_ENABLE) == 0
                        {
                            self.base_state.pwr_mgmt_1 |= bits::LP_EN;
                            let result = self.write_reg::<bank0::Bank>(
                                bank0::PWR_MGMT_1,
                                self.base_state.pwr_mgmt_1,
                            );
                            self.base_state.wake_state |= CHIP_LP_ENABLE;
                            return result;
                        }
                    } else {
                        // lp_en off
                        if (self.base_state.wake_state & CHIP_LP_ENABLE) != 0 {
                            self.base_state.pwr_mgmt_1 &= !bits::LP_EN;
                            let result = self.write_reg::<bank0::Bank>(
                                bank0::PWR_MGMT_1,
                                self.base_state.pwr_mgmt_1,
                            );
                            self.base_state.wake_state &= !CHIP_LP_ENABLE;
                            // After writing the bit wait 100 Micro Seconds
                            self.delay.delay_us(100);
                            return result;
                        }
                    }
                }
            }
            _ => {} // No action for unknown function
        }
        Ok(())
    }

    /// Get chip power state
    pub fn get_chip_power_state(&self) -> u8 {
        self.base_state.wake_state
    }

    /// Wake up MEMS
    pub fn wakeup_mems(&mut self) -> Result<(), Icm20948Error> {
        let mut result = self.set_chip_power_state(CHIP_AWAKE, 1);

        if self.base_state.lp_en_support == SERIAL_INTERFACE_SPI {
            self.base_state.pwr_mgmt_1 |= bits::I2C_IF_DIS;
            result = result
                .and(self.write_reg::<bank0::Bank>(bank0::USER_CTRL, self.base_state.pwr_mgmt_1));
        }

        // FIXME, should set up according to sensor/engines enabled
        let data = 0x47;
        result = result.and(self.write_regs::<bank0::Bank>(bank0::PWR_MGMT_2, &[data]));

        if self.base_state.firmware_loaded {
            self.base_state.pwr_mgmt_1 |= bits::DMP_EN | bits::FIFO_EN;
            result = result
                .and(self.write_reg::<bank0::Bank>(bank0::USER_CTRL, self.base_state.pwr_mgmt_1));
        }

        result = result.and(self.set_chip_power_state(CHIP_LP_ENABLE, 1));

        result
    }

    /// Enable or disable the Sleep mode
    pub fn set_sleep(&mut self, sleep: bool) -> Result<(), Icm20948Error> {
        if sleep {
            self.modify_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |x| x | bits::SLEEP)?;
        } else {
            self.modify_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |x| x & !bits::SLEEP)?;
        }
        Ok(())
    }

    /// Sleep MEMS
    pub fn sleep_mems(&mut self) -> Result<(), Icm20948Error> {
        let data = 0x7F; // All sensors off
        self.write_reg::<bank0::Bank>(bank0::PWR_MGMT_2, data)?;

        self.set_chip_power_state(CHIP_AWAKE, 0)?;

        Ok(())
    }

    /// Set DMP start address
    pub fn set_dmp_address(&mut self, dmp_address: u16) -> Result<(), Icm20948Error> {
        let dmp_cfg = [(dmp_address >> 8) as u8, (dmp_address & 0xff) as u8];
        self.write_mems_regs::<bank2::Bank>(bank2::PRGM_START_ADDRH, &dmp_cfg)
            .map_err(|_| Icm20948Error::InterfaceError)?;

        Ok(())
    }

    /// Setup secondary I2C bus
    pub fn set_secondary(&mut self) -> Result<(), Icm20948Error> {
        let mut result =
            self.write_mems_reg::<bank3::Bank>(bank3::I2C_MST_CTRL, bits::I2C_MST_P_NSR);
        result = result.and(self.write_mems_reg::<bank3::Bank>(
            bank3::I2C_MST_ODR_CONFIG,
            data_defs::MIN_MST_ODR_CONFIG,
        ));

        result
    }

    /// Enter duty cycle mode (low power)
    pub fn enter_duty_cycle_mode(&mut self) -> Result<(), Icm20948Error> {
        // Secondary cycle mode, accel cycle mode and gyro cycle mode
        let data = bits::I2C_MST_CYCLE | bits::ACCEL_CYCLE | bits::GYRO_CYCLE;
        self.base_state.chip_lp_ln_mode = CHIP_LOW_POWER_ICM20948;
        self.write_mems_reg::<bank0::Bank>(bank0::LP_CONFIG, data)
    }

    /// Enter low noise mode
    pub fn enter_low_noise_mode(&mut self) -> Result<(), Icm20948Error> {
        // Only secondary cycle mode
        let data = bits::I2C_MST_CYCLE;

        self.base_state.chip_lp_ln_mode = CHIP_LOW_NOISE_ICM20948;
        self.write_mems_reg::<bank0::Bank>(bank0::LP_CONFIG, data)?;
        Ok(())
    }

    /// Enter low power mode I2C
    pub fn enter_i2c_low_power_mode(&mut self) -> Result<(), Icm20948Error> {
        // Only secondary cycle mode
        self.base_state.chip_lp_ln_mode = CHIP_LOW_POWER_WOI2C_ICM20948;
        self.modify_mems_reg::<bank0::Bank, _>(bank0::LP_CONFIG, |x| x | bits::I2C_MST_CYCLE)?;
        Ok(())
    }

    /// Setup for low power or high performance mode
    pub fn set_lowpower_or_highperformance(
        &mut self,
        lowpower_en: u8,
    ) -> Result<(), Icm20948Error> {
        if lowpower_en != 0 {
            self.enter_duty_cycle_mode()
        } else {
            self.enter_low_noise_mode()
        }
    }

    /// Force low-noise mode by clearing accel/gyro cycle bits and ensuring LP_EN is off.
    pub fn force_low_noise(&mut self) -> Result<(), Icm20948Error> {
        self.modify_mems_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |x| x & !bits::LP_EN)?;
        let mut lp_cfg = self.read_mems_reg::<bank0::Bank>(bank0::LP_CONFIG)?;
        lp_cfg &= !(bits::ACCEL_CYCLE | bits::GYRO_CYCLE);
        lp_cfg |= bits::I2C_MST_CYCLE;
        self.write_mems_reg::<bank0::Bank>(bank0::LP_CONFIG, lp_cfg)?;
        Ok(())
    }

    /// Enable or disable hardware sensors
    /// bit_mask: 0x01 = gyro, 0x02 = accel, 0x04 = pressure
    /// 0x80 = override all sensors
    pub fn enable_hw_sensors(&mut self, bit_mask: u8) -> Result<(), Icm20948Error> {
        let mut result = Ok(());

        if (self.base_state.pwr_mgmt_2
            == (bits::PWR_ACCEL_STBY | bits::PWR_GYRO_STBY | bits::PWR_PRESSURE_STBY))
            || (bit_mask & 0x80) != 0
        {
            // All sensors off or override is on
            self.base_state.pwr_mgmt_2 = 0; // Zero means all sensors are on

            // Turn off accel if not requested
            if (bit_mask & 0x02) == 0 {
                self.base_state.pwr_mgmt_2 |= bits::PWR_ACCEL_STBY;
            }

            // Turn off gyro if not requested
            if (bit_mask & 0x01) == 0 {
                self.base_state.pwr_mgmt_2 |= bits::PWR_GYRO_STBY;
            }

            // Turn off pressure if not requested
            if (bit_mask & 0x04) == 0 {
                self.base_state.pwr_mgmt_2 |= bits::PWR_PRESSURE_STBY;
            }

            result =
                self.write_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_2, self.base_state.pwr_mgmt_2);
        }

        // Configure compass - depends on additional functions
        if (bit_mask & SECONDARY_COMPASS_AVAILABLE) != 0 {
            // resume_akm - needs implementation
        } else {
            // suspend_akm - needs implementation
        }

        result
    }

    /// Set gyro divider
    pub fn set_gyro_divider(&mut self, div: u8) -> Result<(), Icm20948Error> {
        self.base_state.gyro_divider = div as u16;
        self.write_mems_reg::<bank2::Bank>(bank2::GYRO_SMPLRT_DIV, div)
    }

    /// Get gyro divider
    pub fn get_gyro_divider(&self) -> u8 {
        self.base_state.gyro_divider as u8
    }

    /// Set secondary divider
    pub fn set_secondary_divider(&mut self, div: u8) -> Result<(), Icm20948Error> {
        self.base_state.secondary_divider = div;
        self.write_mems_reg::<bank3::Bank>(bank3::I2C_MST_ODR_CONFIG, div)?;
        Ok(())
    }

    /// Get secondary divider
    pub fn get_secondary_divider(&self) -> u8 {
        self.base_state.secondary_divider
    }

    /// Set accel divider
    pub fn set_accel_divider(&mut self, div: u16) -> Result<(), Icm20948Error> {
        self.base_state.accel_divider = div;
        let data = [(div >> 8) as u8, (div & 0xff) as u8];
        self.write_mems_regs::<bank2::Bank>(bank2::ACCEL_SMPLRT_DIV_1, &data)?;
        Ok(())
    }

    /// Get accel divider
    pub fn get_accel_divider(&self) -> u16 {
        self.base_state.accel_divider
    }

    // /// Get ODR in different units
    // pub fn get_odr_in_units(&mut self, odr_in_divider: u16, odr_units: u8) -> u32 {
    //     // Check if gyro is currently enabled
    //     let gyro_is_on = self.is_gyro_enabled();

    //     let us = if pll < 0x80 {
    //         // Correction positive
    //         (odr_in_divider as u64 * 1000000 / 1125) * 1270 /
    //             (1270 + if gyro_is_on { pll as u64 } else { 0 })
    //     } else {
    //         let pll_adj = pll & 0x7F;
    //         // Correction negative
    //         (odr_in_divider as u64 * 1000000 / 1125) * 1270 /
    //             (1270 - if gyro_is_on { pll_adj as u64 } else { 0 })
    //     };

    //     match odr_units {
    //         0 => (us / 1000) as u32, // ODR_IN_Ms - en milisegundos
    //         1 => us as u32,          // ODR_IN_Us - en microsegundos
    //         2 => ((us / 1000) * (32768 / 1125)) as u32, // ODR_IN_Ticks
    //         _ => 0
    //     }
    // }

    /// Set gyro full-scale range
    pub fn set_gyro_fullscale(&mut self, fsr: GyroFullScale) -> Result<(), Icm20948Error> {
        let mut gyro_config_1 = self.read_mems_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1)?;

        gyro_config_1 &= !0x06;
        gyro_config_1 |= match fsr {
            GyroFullScale::Fs250Dps => 0x00,
            GyroFullScale::Fs500Dps => 0x02,
            GyroFullScale::Fs1000Dps => 0x04,
            GyroFullScale::Fs2000Dps => 0x06,
        };

        self.base_state.gyro_fullscale = fsr;
        self.write_mems_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1, gyro_config_1)
    }

    /// Set accel full-scale range
    pub fn set_accel_fullscale(&mut self, fsr: AccelFullScale) -> Result<(), Icm20948Error> {
        let mut accel_config = self.read_mems_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1)?;

        accel_config &= !0x06;
        accel_config |= match fsr {
            AccelFullScale::Fs2G => 0x00,
            AccelFullScale::Fs4G => 0x02,
            AccelFullScale::Fs8G => 0x04,
            AccelFullScale::Fs16G => 0x06,
        };

        self.base_state.accel_fullscale = fsr;
        self.write_mems_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1, accel_config)?;

        // Verify write
        let verify = self.read_mems_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1)?;
        if (verify & 0x06) != (accel_config & 0x06) {
            // We can't easily println here, but we can fail
            // Returning InvalidParameter as a proxy for "Verification Failed"
            return Err(Icm20948Error::InvalidParameter);
        }

        Ok(())
    }

    /// Check if gyro is enabled based on sensors masks
    pub fn is_gyro_enabled(&self) -> bool {
        (self.base_state.wake_state as u32 & INV_NEEDS_GYRO_MASK != 0)
            || (self.base_state.wake_state as u32 & INV_NEEDS_GYRO_MASK1 != 0)
    }

    /// Read accelerometer data from hardware registers
    pub fn accel_read_hw_reg_data(&mut self) -> Result<[i16; 3], Icm20948Error> {
        let mut accel_hw_reg_data = [0i16; 3];
        let mut accel_data = [0u8; 6];
        self.read_mems_regs::<bank0::Bank>(bank0::ACCEL_XOUT_H, &mut accel_data)?;

        // Assign axis values
        accel_hw_reg_data[0] = ((accel_data[0] as i16) << 8) | (accel_data[1] as i16);
        accel_hw_reg_data[1] = ((accel_data[2] as i16) << 8) | (accel_data[3] as i16);
        accel_hw_reg_data[2] = ((accel_data[4] as i16) << 8) | (accel_data[5] as i16);

        Ok(accel_hw_reg_data)
    }

    /// Read gyroscope data from hardware registers
    pub fn gyro_read_hw_reg_data(&mut self) -> Result<[i16; 3], Icm20948Error> {
        let mut gyro_hw_reg_data = [0i16; 3];
        let mut gyro_data = [0u8; 6];
        self.read_mems_regs::<bank0::Bank>(bank0::GYRO_XOUT_H, &mut gyro_data)?;

        // Assign axis values
        gyro_hw_reg_data[0] = ((gyro_data[0] as i16) << 8) | (gyro_data[1] as i16);
        gyro_hw_reg_data[1] = ((gyro_data[2] as i16) << 8) | (gyro_data[3] as i16);
        gyro_hw_reg_data[2] = ((gyro_data[4] as i16) << 8) | (gyro_data[5] as i16);

        Ok(gyro_hw_reg_data)
    }

    /// Read temperature data from hardware registers
    pub fn temperature_read_hw_reg_data(&mut self) -> Result<i16, Icm20948Error> {
        let mut temp_data = [0u8; 2];

        // Read memory registers
        self.read_mems_regs::<bank0::Bank>(bank0::TEMP_OUT_H, &mut temp_data)?;

        // Combine high and low bytes
        let temp_raw = ((temp_data[0] as i16) << 8) | (temp_data[1] as i16);

        Ok(temp_raw)
    }

    /// Read accelerometer hardware offsets (bank1).
    pub fn get_accel_hw_offsets(&mut self) -> Result<[i16; 3], Icm20948Error> {
        let xh = self.read_reg::<bank1::Bank>(bank1::XA_OFFS_H)?;
        let xl = self.read_reg::<bank1::Bank>(bank1::XA_OFFS_L)?;
        let yh = self.read_reg::<bank1::Bank>(bank1::YA_OFFS_H)?;
        let yl = self.read_reg::<bank1::Bank>(bank1::YA_OFFS_L)?;
        let zh = self.read_reg::<bank1::Bank>(bank1::ZA_OFFS_H)?;
        let zl = self.read_reg::<bank1::Bank>(bank1::ZA_OFFS_L)?;
        let x = ((xh as i16) << 8) | (xl as i16);
        let y = ((yh as i16) << 8) | (yl as i16);
        let z = ((zh as i16) << 8) | (zl as i16);
        Ok([x, y, z])
    }

    /// Write accelerometer hardware offsets (bank1).
    pub fn set_accel_hw_offsets(&mut self, offsets: [i16; 3]) -> Result<(), Icm20948Error> {
        self.write_reg::<bank1::Bank>(bank1::XA_OFFS_H, ((offsets[0] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank1::Bank>(bank1::XA_OFFS_L, (offsets[0] & 0xFF) as u8)?;
        self.write_reg::<bank1::Bank>(bank1::YA_OFFS_H, ((offsets[1] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank1::Bank>(bank1::YA_OFFS_L, (offsets[1] & 0xFF) as u8)?;
        self.write_reg::<bank1::Bank>(bank1::ZA_OFFS_H, ((offsets[2] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank1::Bank>(bank1::ZA_OFFS_L, (offsets[2] & 0xFF) as u8)?;
        Ok(())
    }

    /// Read gyroscope hardware offsets (bank2).
    pub fn get_gyro_hw_offsets(&mut self) -> Result<[i16; 3], Icm20948Error> {
        let xh = self.read_reg::<bank2::Bank>(bank2::XG_OFFS_USRH)?;
        let xl = self.read_reg::<bank2::Bank>(bank2::XG_OFFS_USRL)?;
        let yh = self.read_reg::<bank2::Bank>(bank2::YG_OFFS_USRH)?;
        let yl = self.read_reg::<bank2::Bank>(bank2::YG_OFFS_USRL)?;
        let zh = self.read_reg::<bank2::Bank>(bank2::ZG_OFFS_USRH)?;
        let zl = self.read_reg::<bank2::Bank>(bank2::ZG_OFFS_USRL)?;
        let x = ((xh as i16) << 8) | (xl as i16);
        let y = ((yh as i16) << 8) | (yl as i16);
        let z = ((zh as i16) << 8) | (zl as i16);
        Ok([x, y, z])
    }

    /// Write gyroscope hardware offsets (bank2).
    pub fn set_gyro_hw_offsets(&mut self, offsets: [i16; 3]) -> Result<(), Icm20948Error> {
        self.write_reg::<bank2::Bank>(bank2::XG_OFFS_USRH, ((offsets[0] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank2::Bank>(bank2::XG_OFFS_USRL, (offsets[0] & 0xFF) as u8)?;
        self.write_reg::<bank2::Bank>(bank2::YG_OFFS_USRH, ((offsets[1] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank2::Bank>(bank2::YG_OFFS_USRL, (offsets[1] & 0xFF) as u8)?;
        self.write_reg::<bank2::Bank>(bank2::ZG_OFFS_USRH, ((offsets[2] >> 8) & 0xFF) as u8)?;
        self.write_reg::<bank2::Bank>(bank2::ZG_OFFS_USRL, (offsets[2] & 0xFF) as u8)?;
        Ok(())
    }

    pub fn set_clock_source(&mut self, source: u8) -> Result<(), Icm20948Error> {
        let mut pwr_mgmt_1 = self.read_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_1)?;
        pwr_mgmt_1 &= !0x07; // Clear clock source bits
        pwr_mgmt_1 |= source; // Set new clock source
        self.write_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_1, pwr_mgmt_1)?;
        self.base_state.pwr_mgmt_1 = pwr_mgmt_1; // Update base state
        Ok(())
    }

    /// Initialize the device with basic configuration
    pub fn initialize(&mut self) -> Result<(), Icm20948Error> {
        // Reset the device
        self.soft_reset()?;

        // Check WHO_AM_I register
        let whoami = self.get_whoami()?;
        if whoami != 0xEA {
            return Err(Icm20948Error::WhoAmIError);
        }

        self.wakeup()?;
        self.delay.delay_ms(100); // Wait for device to wake up

        self.reset_fifo()?; // Reset FIFO

        // // Wake up the device
        self.set_clock_source(bits::CLK_PLL)?; // Set clock source to PLL with X axis gyroscope reference

        // Configure INT pin
        // 0xF0:
        // Bit 7 (1): Active Low
        // Bit 6 (1): Open Drain
        // Bit 5 (1): Latch until clear
        // Bit 4 (1): Clear on any read
        self.write_regs::<bank0::Bank>(bank0::INT_PIN_CFG, &[0xF0])
            .map_err(|_| Icm20948Error::InterfaceError)?;

        // Enable interrupts
        self.write_regs::<bank0::Bank>(bank0::INT_ENABLE, &[bits::DMP_INT_EN]) // DMP interrupt
            .map_err(|_| Icm20948Error::InterfaceError)?;

        // // Configure FIFO
        self.write_regs::<bank0::Bank>(bank0::FIFO_CFG, &[bits::SINGLE_FIFO_CFG])
            .map_err(|_| Icm20948Error::InterfaceError)?;
        self.write_regs::<bank0::Bank>(bank0::FIFO_RST, &[0x1F]) // Reset all FIFOs
            .map_err(|_| Icm20948Error::InterfaceError)?;
        self.write_regs::<bank0::Bank>(bank0::FIFO_RST, &[0x1E]) // Keep all but Gyro FIFO in reset
            .map_err(|_| Icm20948Error::InterfaceError)?;
        self.write_regs::<bank0::Bank>(bank0::FIFO_EN_1, &[0x00]) // Disable FIFO
            .map_err(|_| Icm20948Error::InterfaceError)?;
        self.write_regs::<bank0::Bank>(bank0::FIFO_EN_2, &[0x00]) // Disable FIFO 2
            .map_err(|_| Icm20948Error::InterfaceError)?;

        // Setup MEMS properties
        self.base_state.chip_lp_ln_mode = 2;
        self.base_state.lp_en_support = 1;
        self.set_gyro_divider(FIFO_DIVIDER)?; // 1125Hz/20 = 56.25Hz
        self.set_accel_divider(FIFO_DIVIDER as u16)?; // 1125Hz/20 = 56.25Hz

        // Configure accel and gyro scales
        self.set_accel_fullscale(AccelFullScale::Fs2G)?; // ±2g
        self.set_gyro_fullscale(GyroFullScale::Fs2000Dps)?; // ±2000dps

        // Activar accel y giroscopio
        self.enable_hw_sensors(0x83)?; // Enable accel and gyro

        // Set low power or low noise mode
        self.set_lowpower_or_highperformance(0)?; // Low noise mode by default

        // Disable HW temp fix
        let mut data: u8;
        data = self.read_mems_reg::<bank0::Bank>(bank0::HW_FIX_DISABLE)?;
        data |= 0x08;
        self.write_mems_reg::<bank0::Bank>(bank0::HW_FIX_DISABLE, data)?;

        self.base_state.firmware_loaded = false;

        Ok(())
    }

    /// Perform a soft reset
    pub fn soft_reset(&mut self) -> Result<(), Icm20948Error> {
        self.base_state.last_bank_selected = 0xFF; // Reset last bank selected
        self.base_state.last_mems_bank_selected = 0xFF; // Reset last MEMS bank selecte
        self.modify_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |val| val | bits::H_RESET)?;
        self.delay.delay_ms(100); // Wait 100ms for reset to complete
        Ok(())
    }

    /// Get device ID (WHO_AM_I register)
    pub fn get_whoami(&mut self) -> Result<u8, Icm20948Error> {
        self.read_mems_reg::<bank0::Bank>(bank0::WHO_AM_I)
    }

    /// Verifica si el firmware DMP está cargado
    pub fn is_dmp_firmware_loaded(&self) -> bool {
        self.base_state.firmware_loaded
    }

    /// Establece el estado del firmware
    pub fn set_firmware_loaded(&mut self, loaded: bool) {
        self.base_state.firmware_loaded = loaded;
    }

    /// Despierta el dispositivo si está en modo sleep
    pub fn wakeup(&mut self) -> Result<(), Icm20948Error> {
        let mut reg = self.read_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG)?;

        if (reg & (bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR))
            != (bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR)
        {
            reg |= bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR;
        } else {
            reg &= !(bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR);
        }

        self.write_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG, reg)?;

        reg = self.read_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_1)?;

        if (reg & bits::INT_BYPASS_EN) != bits::INT_BYPASS_EN {
            reg |= bits::INT_BYPASS_EN;
        } else {
            reg &= !bits::INT_BYPASS_EN;
        }

        self.write_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_1, reg)?;

        Ok(())
    }

    /// Verificar si el DMP está habilitado
    pub fn is_dmp_enabled(&self) -> bool {
        self.base_state.dmp_enabled
    }

    /// Verificar si el firmware está cargado
    pub fn is_firmware_loaded(&self) -> bool {
        self.base_state.firmware_loaded
    }

    /// Establecer el estado de habilitación del DMP
    pub fn set_dmp_enabled(&mut self, enabled: bool) {
        self.base_state.dmp_enabled = enabled;
    }

    /// Obtiene la escala completa del giroscopio actual
    pub fn get_gyro_fullscale(&mut self) -> Result<GyroFullScale, Icm20948Error> {
        // Leemos el valor actual
        let buffer = self.read_mems_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1)?;

        // Extraemos los bits de escala
        let scale = (buffer & bits::GYRO_FS_SEL) >> 1;

        // Guardamos el valor en el estado base
        self.base_state.gyro_fullscale = GyroFullScale::from(scale);

        // Convertimos a la enumeración
        Ok(self.base_state.gyro_fullscale)
    }

    /// Obtiene la escala completa del acelerómetro actual
    pub fn get_accel_fullscale(&mut self) -> Result<AccelFullScale, Icm20948Error> {
        // Leemos el valor actual
        let buffer = self.read_mems_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1)?;

        // Extraemos los bits de escala
        let scale = (buffer & bits::ACCEL_FS_SEL) >> 1;

        // Convertimos a la enumeración
        Ok(AccelFullScale::from(scale))
    }

    /// Convierte un valor raw de acelerómetro a g según la escala configurada
    pub fn convert_accel_to_g(&self, raw_value: i16) -> f32 {
        let scale_factor = match self.base_state.accel_fullscale {
            AccelFullScale::Fs2G => 16384.0,
            AccelFullScale::Fs4G => 8192.0,
            AccelFullScale::Fs8G => 4096.0,
            AccelFullScale::Fs16G => 2048.0,
        };

        raw_value as f32 / scale_factor
    }

    /// Convierte un valor raw de giroscopio a grados/segundo según la escala configurada
    pub fn convert_gyro_to_dps(&self, raw_value: i16) -> f32 {
        let scale_factor = match self.base_state.gyro_fullscale {
            GyroFullScale::Fs250Dps => 131.0,
            GyroFullScale::Fs500Dps => 65.5,
            GyroFullScale::Fs1000Dps => 32.8,
            GyroFullScale::Fs2000Dps => 16.4,
        };

        raw_value as f32 / scale_factor
    }

    /// Lee los datos del acelerómetro
    pub fn get_accel(&mut self) -> Result<AccelData, Icm20948Error> {
        let mut accel_data: [u8; 6] = [0; 6];
        self.read_mems_regs::<bank0::Bank>(bank0::ACCEL_XOUT_H, &mut accel_data)?;

        let accel_x = i16::from_be_bytes([accel_data[0], accel_data[1]]);
        let accel_y = i16::from_be_bytes([accel_data[2], accel_data[3]]);
        let accel_z = i16::from_be_bytes([accel_data[4], accel_data[5]]);

        Ok(AccelData {
            x: self.convert_accel_to_g(accel_x),
            y: self.convert_accel_to_g(accel_y),
            z: self.convert_accel_to_g(accel_z),
            timestamp_us: SystemTimeSource.get_timestamp_us(),
        })
    }

    /// Activar el modo de bajo consumo
    pub fn low_power(&mut self, en: bool) -> Result<(), Icm20948Error> {
        if en {
            self.modify_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |x| x | bits::LP_EN)?;
        } else {
            self.modify_reg::<bank0::Bank, _>(bank0::PWR_MGMT_1, |x| x & !bits::LP_EN)?;
            // Also ensure cycling is disabled in LP_CONFIG (Bit 6: I2C_MST_CYCLE, Bit 5: ACCEL_CYCLE, Bit 4: GYRO_CYCLE)
            self.modify_reg::<bank0::Bank, _>(bank0::LP_CONFIG, |x| x & !0x70)?;
        }
        Ok(())
    }

    /// Configurar interrupciones
    pub fn config_interrupt(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let mut reg = self.read_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG)?;
        if enable {
            reg |= bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR;
        } else {
            reg &= !(bits::INT1_ACTL | bits::INT_ANYRD_2CLEAR);
        }
        self.write_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG, reg)?;

        Ok(())
    }

    /// Habilitar interrupciones
    pub fn enable_interrupt(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        self.write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE_1, if enable { 0x01 } else { 0x00 })?;
        Ok(())
    }

    /// Activar bypass I2C
    pub fn enable_bypass_i2c(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        // Para activar bypass I2C:
        // 1. Desactivar el I2C Master si está activado
        // 2. Establecer el bit de bypass
        // 3. Para desactivar, limpiamos el bit de bypass

        // Asegurarnos que estamos en el banco 0 para acceder a estos registros
        self.set_bank(0)?;

        if enable {
            // Leer el registro USER_CTRL
            let user_ctrl = self.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)?;

            // Desactivar I2C Master si está activado
            if (user_ctrl & bits::I2C_MST_EN) != 0 {
                self.write_mems_reg::<bank0::Bank>(
                    bank0::USER_CTRL,
                    user_ctrl & !bits::I2C_MST_EN,
                )?;
                // Esperar a que el cambio surta efecto
                self.delay.delay_ms(10);
            }
        }

        // Leer el registro INT_PIN_CFG
        let mut reg = self.read_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG)?;

        if enable {
            reg |= bits::INT_BYPASS_EN;
        } else {
            reg &= !bits::INT_BYPASS_EN;

            // Si desactivamos bypass, podemos necesitar reactivar el I2C Master
            let user_ctrl = self.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)?;
            self.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL, user_ctrl | bits::I2C_MST_EN)?;
        }

        // Escribir el registro INT_PIN_CFG
        self.write_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG, reg)?;

        // Esperar a que los cambios surtan efecto
        self.delay.delay_ms(5);

        Ok(())
    }

    /// Activar DMP
    pub fn enable_dmp(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        self.modify_mems_reg::<bank0::Bank, _>(bank0::USER_CTRL, |val| {
            if enable {
                val | bits::DMP_EN
            } else {
                val & !bits::DMP_EN
            }
        })?;
        self.base_state.dmp_enabled = enable;
        Ok(())
    }

    /// Leer los datos de temperatura sin procesar
    pub fn read_temp_raw(&mut self) -> Result<i16, Icm20948Error> {
        let mut buffer = [0u8; 2];
        self.read_mems_regs::<bank0::Bank>(bank0::TEMP_OUT_H, &mut buffer)?;
        let raw_temp = ((buffer[0] as i16) << 8) | (buffer[1] as i16);
        Ok(raw_temp)
    }

    /// Obtener la escala del acelerómetro
    pub fn get_accel_scale(&mut self) -> f32 {
        match self.base_state.accel_fullscale {
            AccelFullScale::Fs2G => gravity::GRAVITY_MSS / 16384.0,
            AccelFullScale::Fs4G => gravity::GRAVITY_MSS / 8192.0,
            AccelFullScale::Fs8G => gravity::GRAVITY_MSS / 4096.0,
            AccelFullScale::Fs16G => gravity::GRAVITY_MSS / 2048.0,
        }
    }

    /// Obtener la escala del giroscopio
    pub fn get_gyro_scale(&mut self) -> f32 {
        match self.base_state.gyro_fullscale {
            GyroFullScale::Fs250Dps => 250.0 / 32768.0,
            GyroFullScale::Fs500Dps => 500.0 / 32768.0,
            GyroFullScale::Fs1000Dps => 1000.0 / 32768.0,
            GyroFullScale::Fs2000Dps => 2000.0 / 32768.0,
        }
    }

    /// Lee los datos raw del acelerómetro y los devuelve como valores en G
    pub fn read_accelerometer(&mut self) -> Result<[f32; 3], Icm20948Error> {
        let raw_data = self.accel_read_hw_reg_data()?;
        eprintln!(
            "Raw accel data: {:?}, Full scale: {:?}",
            raw_data, self.base_state.accel_fullscale
        );
        Ok(accel_raw_to_g(raw_data, self.base_state.accel_fullscale))
    }

    /// Lee los datos raw del giroscopio y los devuelve como valores en grados/segundo
    pub fn read_gyroscope(&mut self) -> Result<[f32; 3], Icm20948Error> {
        let raw_data = self.gyro_read_hw_reg_data()?;
        // eprintln!("Raw gyro data: {:?}, Full scale: {:?}", raw_data, self.base_state.gyro_fullscale);
        Ok(gyro_raw_to_dps(raw_data, self.base_state.gyro_fullscale))
    }

    /// Lee el valor raw de temperatura y lo convierte a grados Celsius
    pub fn read_temperature(&mut self) -> Result<f32, Icm20948Error> {
        let raw_temp = self.read_temp_raw()?;
        // eprintln!("Raw temperature data: {:?}", raw_temp);
        Ok(temp_raw_to_celsius(raw_temp))
    }
}
