//! Implementación del magnetómetro auxiliar (compass) para ICM20948
//!
//! Este módulo proporciona soporte para los magnetómetros AKM conectados como dispositivos
//! secundarios al ICM20948 a través de su interfaz I2C auxiliar.

use crate::base::{SystemTimeSource, TimeSource};
use crate::interface::Interface;
use crate::{device::Icm20948, device::Icm20948Error, types::bits};
use core::default::Default;
use embedded_hal::blocking::delay::DelayMs;

// Import register
use crate::register::ak_reg;
use crate::register::registers::{bank0, bank3};
use crate::register::slave_reg;
use crate::types::ak_val;
use crate::types::scale_factor;
use crate::types::slv_bits;

/// Tamaño de los datos del magnetómetro (2 bytes por eje x 3 ejes)
pub const COMPASS_DATA_SZ: usize = 6;

pub mod compass_rd {
    // AK09916 CNTL2 modes
    pub const MODE_POWERDOWN: u8 = 0x00;
    pub const MODE_SINGLE: u8 = 0x01;
    pub const MODE_CONT_10HZ: u8 = 0x02;
    pub const MODE_CONT_20HZ: u8 = 0x04;
    pub const MODE_CONT_50HZ: u8 = 0x06;
    pub const MODE_CONT_100HZ: u8 = 0x08;
}

/// Tipos de magnetómetros compatibles
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CompassType {
    /// Sin magnetómetro
    None = 0,
    /// AKM AK09911
    AK09911 = 1,
    /// AKM AK09912
    AK09912 = 2,
    /// AKM AK09916 (el más común en dispositivos modernos)
    AK09916 = 3,
    /// AKM AK08963
    AK08963 = 4,
}

/// Estado del magnetómetro
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CompassState {
    /// Magnetómetro reseteado/inactivo
    Reset = 0,
    /// Magnetómetro inicializado
    Initialized = 1,
    /// Magnetómetro configurado y listo
    Setup = 2,
}

/// Datos de calibración del magnetómetro
#[derive(Debug, Clone, Copy)]
pub struct CompassCalibrationData {
    /// Valores de sensibilidad del fabricante
    pub sensitivity: [u8; 3],
    /// Matrix de montaje (9 elementos)
    pub mounting_matrix: [i8; 9],
    /// Escala actual
    pub scale: u8,
    /// Límite inferior de auto-test
    pub st_lower: [i16; 3],
    /// Límite superior de auto-test
    pub st_upper: [i16; 3],
}

impl Default for CompassCalibrationData {
    fn default() -> Self {
        Self {
            sensitivity: [128, 128, 128], // Valores predeterminados cuando no hay ajuste
            mounting_matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1], // Identidad por defecto
            scale: 0,
            st_lower: [-200, -200, -1000], // AK09916 default
            st_upper: [200, 200, -200],    // AK09916 default
        }
    }
}

/// Configuración del magnetómetro/compás secundario
#[derive(Debug)]
pub struct CompassConfig {
    /// Tipo de compás conectado
    pub compass_type: CompassType,
    /// Dirección I2C del compás
    pub compass_i2c_addr: u8,
    /// Estado del compás
    pub compass_state: CompassState,
    /// Datos de calibración
    pub calibration: CompassCalibrationData,
    /// Modo de operación
    pub mode: u8,
}

impl Clone for CompassConfig {
    fn clone(&self) -> Self {
        Self {
            compass_type: self.compass_type,
            compass_i2c_addr: self.compass_i2c_addr,
            compass_state: self.compass_state,
            calibration: self.calibration,
            mode: self.mode,
        }
    }
}

impl Default for CompassConfig {
    fn default() -> Self {
        Self {
            compass_type: CompassType::None,
            compass_i2c_addr: 0,
            compass_state: CompassState::Reset,
            calibration: CompassCalibrationData::default(),
            mode: compass_rd::MODE_SINGLE,
        }
    }
}

/// Datos del magnetómetro/compás en unidades físicas (micro teslas)
#[derive(Debug, Clone, Copy)]
pub struct CompassData {
    /// Campo magnético en eje X (µT)
    pub x: f32,
    /// Campo magnético en eje Y (µT)
    pub y: f32,
    /// Campo magnético en eje Z (µT)
    pub z: f32,
    /// Timestamp (µs)
    pub timestamp_us: u64,
    /// Nivel de precisión (0-3)
    pub accuracy: u8,
}

impl Default for CompassData {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            timestamp_us: 0,
            accuracy: 0,
        }
    }
}

/// Canales I2C secundarios disponibles
#[derive(Debug, Clone, Copy)]
pub enum SecondaryI2cChannel {
    Slave0 = 0,
    Slave1 = 1,
    Slave2 = 2,
    Slave3 = 3,
    Slave4 = 4,
}

/// Implementación para el soporte de compás
impl<I, D, E> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
{
    /// Registra un compás auxiliar con el driver
    pub fn register_aux_compass(
        &mut self,
        compass_type: CompassType,
        compass_i2c_addr: u8,
    ) -> Result<(), Icm20948Error> {
        // Configurar estructura de compás
        let mut compass_config = CompassConfig {
            compass_type,
            compass_i2c_addr,
            compass_state: CompassState::Initialized,
            ..Default::default()
        };

        // Configurar matriz de montaje según el tipo de compás
        match compass_type {
            CompassType::AK09911 => {
                compass_config.calibration.mounting_matrix = [-1, 0, 0, 0, -1, 0, 0, 0, 1];
            }
            CompassType::AK09912 => {
                compass_config.calibration.mounting_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1];
            }
            CompassType::AK08963 => {
                compass_config.calibration.mounting_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1];
            }
            CompassType::AK09916 => {
                compass_config.calibration.mounting_matrix = [1, 0, 0, 0, -1, 0, 0, 0, -1];
            }
            _ => return Err(Icm20948Error::InvalidParameter),
        }

        // Configurar límites de self-test
        match compass_type {
            CompassType::AK09916 => {
                compass_config.calibration.st_lower = [-200, -200, -1000];
                compass_config.calibration.st_upper = [200, 200, -200];
            }
            CompassType::AK09911 => {
                compass_config.calibration.st_lower = [-30, -30, -400];
                compass_config.calibration.st_upper = [30, 30, -50];
            }
            CompassType::AK09912 => {
                compass_config.calibration.st_lower = [-200, -200, -1600];
                compass_config.calibration.st_upper = [200, 200, -400];
            }
            CompassType::AK08963 => {
                compass_config.calibration.st_lower = [-200, -200, -3200];
                compass_config.calibration.st_upper = [200, 200, -800];
            }
            _ => return Err(Icm20948Error::InvalidParameter),
        }

        // Guardar configuración en el estado del dispositivo
        self.enable_compass_config(compass_config)?;

        Ok(())
    }

    /// Guarda la configuración del compás
    pub fn enable_compass_config(&mut self, config: CompassConfig) -> Result<(), Icm20948Error> {
        self.base_state.compass_config = Some(config); // Guardar la configuración en el estado del dispositivo

        Ok(())
    }

    /// Inicializa el compás conectado al ICM20948
    pub fn setup_compass(
        &mut self,
        compass_type: CompassType,
        compass_i2c_addr: u8,
    ) -> Result<CompassConfig, Icm20948Error> {
        // Registramos primero el compás
        self.register_aux_compass(compass_type, compass_i2c_addr)?;
        // Inicializar la configuración secundaria I2C
        self.init_secondary_i2c()?;

        // Esperar un momento para que el bus I2C se estabilice
        self.delay.delay_ms(20u32);

        // Resetear el compás
        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            compass_i2c_addr,
            ak_reg::AK09916_CNTL3,
            0x01,
        )?;

        let mut retries = 10;
        while retries > 0 {
            let mut whoami_data = [0u8];
            self.read_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                compass_i2c_addr,
                ak_reg::WIA,
                &mut whoami_data,
            )?;

            // Verificar que es un compás AKM
            if whoami_data[0] != ak_val::WIA_VAL {
                // Probar a resetear el Master I2C
                self.reset_master_i2c()?;
                retries -= 1;
            } else {
                break;
            }
            self.delay.delay_ms(20u32);
        }
        if retries == 0 {
            return Err(Icm20948Error::WhoAmIError);
        }

        println!("Compás conectado: {:?} ({:?})", compass_type, retries);

        // Crear una configuración de compás
        let mut compass_config = CompassConfig {
            compass_type,
            compass_i2c_addr,
            compass_state: CompassState::Initialized,
            calibration: CompassCalibrationData::default(),
            mode: compass_rd::MODE_SINGLE,
        };

        // Leer valores de sensibilidad desde el propio AKM
        if compass_type != CompassType::AK09916 {
            // AK09916 no tiene ROM de sensibilidad
            // Modo ROM para leer sensibilidad
            let mode_reg = match compass_type {
                CompassType::AK09911 => ak_reg::AK09911_CNTL2,
                CompassType::AK09912 => ak_reg::AK09912_CNTL2,
                _ => ak_reg::MODE,
            };

            let mode_val = match compass_type {
                CompassType::AK09911 => ak_val::AK09911_FUSE_ROM,
                CompassType::AK09912 => ak_val::AK09912_FUSE_ROM,
                _ => ak_val::FUSE_ROM_ACCESS,
            };

            // Poner en modo ROM
            self.write_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                compass_i2c_addr,
                mode_reg,
                mode_val,
            )?;

            // Leer sensibilidad
            let sens_reg = match compass_type {
                CompassType::AK09911 => ak_reg::AK09911_SENSITIVITY,
                CompassType::AK09912 => ak_reg::AK09912_SENSITIVITY,
                _ => ak_reg::AK8963_SENSITIVITY,
            };

            let mut sens_data = [0u8; 3];
            self.read_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                compass_i2c_addr,
                sens_reg,
                &mut sens_data,
            )?;
            compass_config.calibration.sensitivity = sens_data;
        }

        // Establecer modo power down
        let mode_reg = match compass_type {
            CompassType::AK09916 => ak_reg::AK09916_CNTL2,
            CompassType::AK09911 => ak_reg::AK09911_CNTL2,
            CompassType::AK09912 => ak_reg::AK09912_CNTL2,
            _ => ak_reg::MODE,
        };

        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            compass_i2c_addr,
            mode_reg,
            ak_val::SINGLE_MEASURE,
        )?;

        // Configurar para modo cíclico si es AK09912
        if compass_type == CompassType::AK09912 {
            // Aplicar filtro de supresión de ruido
            self.write_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                compass_i2c_addr,
                ak_reg::AK09912_CNTL1,
                0x20, // NSF = 1 (filtro bajo)
            )?;
        }

        // Actualizar estado
        compass_config.compass_state = CompassState::Setup;

        // Guardar la configuración
        self.enable_compass_config(compass_config.clone())?;

        Ok(compass_config)
    }

    /// Realiza una lectura del compás
    pub fn read_compass(&mut self, timeout_ms: u32) -> Result<CompassData, Icm20948Error> {
        let timeout_ms = if timeout_ms == 0 { 1000 } else { timeout_ms };
        if self.base_state.compass_config.is_none()
            || self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_state
                != CompassState::Setup
        {
            return Err(Icm20948Error::InvalidParameter);
        }

        if self.base_state.compass_config.as_mut().unwrap().mode != compass_rd::MODE_SINGLE {
            // Establecer modo de medida única
            let mode_reg = match self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_type
            {
                CompassType::AK09916 => ak_reg::AK09916_CNTL2,
                CompassType::AK09911 => ak_reg::AK09911_CNTL2,
                CompassType::AK09912 => ak_reg::AK09912_CNTL2,
                _ => compass_rd::MODE_SINGLE,
            };

            self.write_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                self.base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .compass_i2c_addr,
                mode_reg,
                ak_val::SINGLE_MEASURE,
            )?;
        }

        // Leer registro de estado para verificar si hay datos listos
        let status_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_STATUS1,
            CompassType::AK09911 => ak_reg::AK09911_STATUS1,
            CompassType::AK09912 => ak_reg::AK09912_STATUS1,
            _ => ak_reg::STATUS,
        };

        // Esperar a que los datos estén listos (DRDY)
        let mut count = 0;
        while count < timeout_ms {
            let mut status = [0u8];
            self.read_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                self.base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .compass_i2c_addr,
                status_reg,
                &mut status,
            )?;

            // Verificar bit DRDY
            if (status[0] & ak_val::DRDY) == 0 {
                // Datos no listos
                count += 1;
                std::thread::sleep(std::time::Duration::from_millis(1));
                continue;
            }
            break;
        }

        if count == timeout_ms {
            return Err(Icm20948Error::Timeout);
        }

        // Leer datos de medición
        let data_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_MEASURE_DATA,
            CompassType::AK09911 => ak_reg::AK09911_MEASURE_DATA,
            CompassType::AK09912 => ak_reg::AK09912_MEASURE_DATA,
            _ => ak_reg::MEASURE_DATA,
        };

        let mut raw_data = [0u8; 6];
        self.read_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            data_reg,
            &mut raw_data,
        )?;

        // Convertir a valores de 16 bits
        let mut mag_values = [0i16; 3];
        for i in 0..3 {
            mag_values[i] = (raw_data[i * 2] as i16) | ((raw_data[i * 2 + 1] as i16) << 8);
        }

        // Aplicar sensibilidad y convertir a µT
        let scale_factor = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => scale_factor::AK09916,
            CompassType::AK09911 => scale_factor::AK09911,
            CompassType::AK09912 => scale_factor::AK09912,
            CompassType::AK08963 => {
                if self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .scale
                    == 0
                {
                    scale_factor::AK8963_14BIT
                } else {
                    scale_factor::AK8963_16BIT
                }
            }
            _ => 0,
        };

        // Aplicar matriz de montaje y sensibilidad
        let mut mag_data = CompassData::default();

        // Calcular los valores compensados
        let mut compensated = [0f32; 3];
        for (i, comp_val) in compensated.iter_mut().enumerate() {
            let mut tmp_val = 0;
            for (j, &mag_val) in mag_values.iter().enumerate() {
                let sens_adj = (self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .sensitivity[j] as i32
                    + 128) as f32;
                let matrix_val = self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .mounting_matrix[i * 3 + j] as f32;
                tmp_val += (mag_val as f32 * matrix_val * sens_adj / 256.0) as i32;
            }
            *comp_val = (tmp_val as f32 * scale_factor as f32) / (1u64 << 30) as f32;
        }

        mag_data.x = compensated[0];
        mag_data.y = compensated[1];
        mag_data.z = compensated[2];
        mag_data.accuracy = 3; // Alta precisión
        mag_data.timestamp_us = SystemTimeSource.get_timestamp_us();

        // Leer datos de estado
        let status_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_STATUS2,
            CompassType::AK09911 => ak_reg::AK09911_STATUS2,
            CompassType::AK09912 => ak_reg::AK09912_STATUS2,
            _ => ak_reg::STATUS,
        };

        let mut status_data = [0u8; 1];
        self.read_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            status_reg,
            &mut status_data,
        )?;

        // Verificar si hay errores de medición
        if (status_data[0] & 0x08) != 0 {
            return Err(Icm20948Error::Overflow);
        }

        Ok(mag_data)
    }

    /// Verifica si el compás está conectado
    pub fn compass_is_connected(&self) -> bool {
        self.base_state.compass_config.is_some()
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_state
                == CompassState::Setup
    }

    /// Configura el modo de operación del compás
    pub fn set_compass_mode(&mut self, mode: u8) -> Result<(), Icm20948Error> {
        if self.base_state.compass_config.is_none()
            || self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_state
                != CompassState::Setup
        {
            return Err(Icm20948Error::InvalidParameter);
        }

        // Guardar el modo en la configuración del compás
        self.base_state.compass_config.as_mut().unwrap().mode = mode;

        // Configurar el modo en el registro correspondiente
        let mode_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_CNTL2,
            CompassType::AK09911 => ak_reg::AK09911_CNTL2,
            CompassType::AK09912 => ak_reg::AK09912_CNTL2,
            _ => ak_reg::MODE,
        };

        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            mode_reg,
            mode,
        )?;

        Ok(())
    }

    /// Realiza un self-test del compás
    pub fn check_compass_self_test(&mut self) -> Result<bool, Icm20948Error> {
        if self.base_state.compass_config.is_none()
            || self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_state
                != CompassState::Setup
        {
            return Err(Icm20948Error::InvalidParameter);
        }

        // Guardar configuración actual
        let saved_slv_ctrl = [0u8; 2];
        self.set_bank(3)?;
        self.read_regs_raw(slave_reg::I2C_SLV0_CTRL, &mut [saved_slv_ctrl[0]])?;
        self.read_regs_raw(slave_reg::I2C_SLV1_CTRL, &mut [saved_slv_ctrl[1]])?;

        // Deshabilitar temporalmente los esclavos
        self.write_regs_raw(slave_reg::I2C_SLV0_CTRL, &[0])?;
        self.write_regs_raw(slave_reg::I2C_SLV1_CTRL, &[0])?;

        // Poner compás en modo power down
        let mode_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_CNTL2,
            CompassType::AK09911 => ak_reg::AK09911_CNTL2,
            CompassType::AK09912 => ak_reg::AK09912_CNTL2,
            _ => ak_reg::MODE,
        };

        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            mode_reg,
            ak_val::POWER_DOWN,
        )?;

        // Activar self-test si no es AK09911/AK09912/AK09916
        if self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
            != CompassType::AK09911
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_type
                != CompassType::AK09912
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_type
                != CompassType::AK09916
        {
            self.write_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                self.base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .compass_i2c_addr,
                ak_reg::ASTC,
                ak_val::SELF_TEST,
            )?;
        }

        // Configurar modo de self-test
        let st_mode = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_val::AK09916_MODE_ST,
            CompassType::AK09911 => ak_val::AK09916_MODE_ST, // Mismo valor para AK09911
            CompassType::AK09912 => ak_val::AK09916_MODE_ST, // Mismo valor para AK09912
            _ => ak_val::SELF_TEST,
        };
        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            mode_reg,
            st_mode,
        )?;

        // Esperar a que los datos estén listos
        // Normalmente aquí habría un delay entre 10-15ms

        // Verificar bit DRDY
        let status_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_STATUS1,
            CompassType::AK09911 => ak_reg::AK09911_STATUS1,
            CompassType::AK09912 => ak_reg::AK09912_STATUS1,
            _ => ak_reg::STATUS,
        };

        let mut status = [0u8];
        self.read_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            status_reg,
            &mut status,
        )?;

        if (status[0] & ak_val::DRDY) == 0 {
            // Datos no listos, self-test fallido
            return Ok(false);
        }

        // Leer datos de self-test
        let data_reg = match self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
        {
            CompassType::AK09916 => ak_reg::AK09916_MEASURE_DATA,
            CompassType::AK09911 => ak_reg::AK09911_MEASURE_DATA,
            CompassType::AK09912 => ak_reg::AK09912_MEASURE_DATA,
            _ => ak_reg::MEASURE_DATA,
        };

        let mut raw_data = [0u8; 6];
        self.read_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            data_reg,
            &mut raw_data,
        )?;

        // Convertir a valores de 16 bits
        let mut st_values = [0i16; 3];
        for i in 0..3 {
            st_values[i] = (raw_data[i * 2] as i16) | ((raw_data[i * 2 + 1] as i16) << 8);
        }

        // Aplicar sensibilidad
        let shift = if self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
            == CompassType::AK09911
        {
            7
        } else {
            8
        };
        for (i, val) in st_values.iter_mut().enumerate() {
            let sens_adj = (self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .calibration
                .sensitivity[i]
                + 128) as i32;
            *val = ((*val as i32 * sens_adj) >> shift) as i16;
        }

        // Ajustar para AK8963 con 16-bit
        if self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
            == CompassType::AK08963
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .calibration
                .scale
                == 1
        {
            // Doblar los valores para 16-bit mode
            for val in &mut st_values {
                *val <<= 2;
            }
        }

        // Verificar contra límites
        let pass = st_values[0]
            > self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .calibration
                .st_lower[0]
            && st_values[0]
                < self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .st_upper[0]
            && st_values[1]
                > self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .st_lower[1]
            && st_values[1]
                < self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .st_upper[1]
            && st_values[2]
                > self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .st_lower[2]
            && st_values[2]
                < self
                    .base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .calibration
                    .st_upper[2];

        // Limpiar bit de self-test si es necesario
        if self
            .base_state
            .compass_config
            .as_ref()
            .unwrap()
            .compass_type
            != CompassType::AK09911
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_type
                != CompassType::AK09912
            && self
                .base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_type
                != CompassType::AK09916
        {
            self.write_secondary_i2c(
                SecondaryI2cChannel::Slave3,
                self.base_state
                    .compass_config
                    .as_ref()
                    .unwrap()
                    .compass_i2c_addr,
                ak_reg::ASTC,
                0,
            )?;
        }

        // Volver a modo power down
        self.write_secondary_i2c(
            SecondaryI2cChannel::Slave3,
            self.base_state
                .compass_config
                .as_ref()
                .unwrap()
                .compass_i2c_addr,
            mode_reg,
            ak_val::POWER_DOWN,
        )?;

        // Restaurar configuración de esclavos
        self.write_regs_raw(slave_reg::I2C_SLV0_CTRL, &[saved_slv_ctrl[0]])?;
        self.write_regs_raw(slave_reg::I2C_SLV1_CTRL, &[saved_slv_ctrl[1]])?;

        Ok(pass)
    }

    /// Inicializa la interfaz I2C secundaria
    fn init_secondary_i2c(&mut self) -> Result<(), Icm20948Error> {
        self.modify_reg::<bank0::Bank, _>(bank0::INT_PIN_CFG, |val| val & !bits::I2C_BYPASS_EN)?;
        self.modify_reg::<bank3::Bank, _>(bank3::I2C_MST_CTRL, |val| (val & 0xE0) | 0x17)?; // 0x17 = 0b00010111 (I2C_MST_DELAY_CTRL)
        self.modify_reg::<bank0::Bank, _>(bank0::USER_CTRL, |val| val | bits::I2C_MST_EN)?; // Habilitar I2C maestro

        Ok(())
    }

    /// Reset Master I2C
    fn reset_master_i2c(&mut self) -> Result<(), Icm20948Error> {
        self.modify_reg::<bank0::Bank, _>(bank0::USER_CTRL, |val| val | bits::I2C_MST_RST)?;

        Ok(())
    }

    /// Lee datos desde el bus I2C secundario
    fn read_secondary_i2c(
        &mut self,
        channel: SecondaryI2cChannel,
        addr: u8,
        reg: u8,
        data: &mut [u8],
    ) -> Result<(), Icm20948Error> {
        // Determinar los registros según el canal
        let (addr_reg, reg_reg, ctrl_reg, _) = match channel {
            SecondaryI2cChannel::Slave0 => (
                slave_reg::I2C_SLV0_ADDR,
                slave_reg::I2C_SLV0_REG,
                slave_reg::I2C_SLV0_CTRL,
                slave_reg::I2C_SLV0_DO,
            ),
            SecondaryI2cChannel::Slave1 => (
                slave_reg::I2C_SLV1_ADDR,
                slave_reg::I2C_SLV1_REG,
                slave_reg::I2C_SLV1_CTRL,
                slave_reg::I2C_SLV1_DO,
            ),
            SecondaryI2cChannel::Slave2 => (
                slave_reg::I2C_SLV2_ADDR,
                slave_reg::I2C_SLV2_REG,
                slave_reg::I2C_SLV2_CTRL,
                slave_reg::I2C_SLV2_DO,
            ),
            SecondaryI2cChannel::Slave3 => (
                slave_reg::I2C_SLV3_ADDR,
                slave_reg::I2C_SLV3_REG,
                slave_reg::I2C_SLV3_CTRL,
                slave_reg::I2C_SLV3_DO,
            ),
            SecondaryI2cChannel::Slave4 => (
                slave_reg::I2C_SLV4_ADDR,
                slave_reg::I2C_SLV4_REG,
                slave_reg::I2C_SLV4_CTRL,
                slave_reg::I2C_SLV4_DO,
            ),
        };

        // Cambiar al banco 3
        self.set_bank(3)?;

        // Configurar esclavo para leer
        self.write_regs_raw(addr_reg, &[addr | slv_bits::I2C_READ])?;
        self.write_regs_raw(reg_reg, &[reg])?;

        // Calcular bits de control
        let len = data.len().min(15) as u8; // Máximo 15 bytes por transacción
        let ctrl_val = slv_bits::I2C_ENABLE | len;

        self.write_regs_raw(ctrl_reg, &[ctrl_val])?; // Update to use slice for ctrl_val

        // Cambiar al banco 0
        self.set_bank(0)?;

        // Esperar a que la transacción complete
        self.read_regs_raw(slave_reg::EXT_SENS_DATA_00, data)?;

        Ok(())
    }

    /// Escribe datos al bus I2C secundario
    fn write_secondary_i2c(
        &mut self,
        channel: SecondaryI2cChannel,
        addr: u8,
        reg: u8,
        data: u8,
    ) -> Result<(), Icm20948Error> {
        // Determinar los registros según el canal
        let (addr_reg, reg_reg, ctrl_reg, do_reg) = match channel {
            SecondaryI2cChannel::Slave0 => (
                slave_reg::I2C_SLV0_ADDR,
                slave_reg::I2C_SLV0_REG,
                slave_reg::I2C_SLV0_CTRL,
                slave_reg::I2C_SLV0_DO,
            ),
            SecondaryI2cChannel::Slave1 => (
                slave_reg::I2C_SLV1_ADDR,
                slave_reg::I2C_SLV1_REG,
                slave_reg::I2C_SLV1_CTRL,
                slave_reg::I2C_SLV1_DO,
            ),
            SecondaryI2cChannel::Slave2 => (
                slave_reg::I2C_SLV2_ADDR,
                slave_reg::I2C_SLV2_REG,
                slave_reg::I2C_SLV2_CTRL,
                slave_reg::I2C_SLV2_DO,
            ),
            SecondaryI2cChannel::Slave3 => (
                slave_reg::I2C_SLV3_ADDR,
                slave_reg::I2C_SLV3_REG,
                slave_reg::I2C_SLV3_CTRL,
                slave_reg::I2C_SLV3_DO,
            ),
            SecondaryI2cChannel::Slave4 => (
                slave_reg::I2C_SLV4_ADDR,
                slave_reg::I2C_SLV4_REG,
                slave_reg::I2C_SLV4_CTRL,
                slave_reg::I2C_SLV4_DO,
            ),
        };

        // Cambiar al banco 3
        self.set_bank(3)?;

        // Configurar esclavo para escribir
        self.write_regs_raw(addr_reg, &[addr & !slv_bits::I2C_READ])?;
        self.write_regs_raw(reg_reg, &[reg])?;
        self.write_regs_raw(do_reg, &[data])?;

        // Habilitar escritura (1 byte)
        self.write_regs_raw(ctrl_reg, &[slv_bits::I2C_ENABLE | 1])?;

        // Cambiar al banco 0
        self.set_bank(0)?;

        Ok(())
    }
}
