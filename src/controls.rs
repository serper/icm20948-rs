//! Controles básicos para el sensor ICM20948

use crate::device::{Icm20948, Icm20948Error, ACCEL_AVAILABLE, GYRO_AVAILABLE, SECONDARY_COMPASS_AVAILABLE};
use crate::interface::Interface;
use crate::register::registers::{bank0, bank2};
use crate::types::bits;
use embedded_hal::blocking::delay::DelayMs;

/// Factores de conversión para los diferentes rangos de escala completa del acelerómetro
pub const ACCEL_SCALE_FACTOR_2G: f32 = 16384.0;  // LSB/g
pub const ACCEL_SCALE_FACTOR_4G: f32 = 8192.0;   // LSB/g
pub const ACCEL_SCALE_FACTOR_8G: f32 = 4096.0;   // LSB/g
pub const ACCEL_SCALE_FACTOR_16G: f32 = 2048.0;  // LSB/g

/// Factores de conversión para los diferentes rangos de escala completa del giroscopio
pub const GYRO_SCALE_FACTOR_250DPS: f32 = 131.0;   // LSB/(°/s)
pub const GYRO_SCALE_FACTOR_500DPS: f32 = 65.5;    // LSB/(°/s)
pub const GYRO_SCALE_FACTOR_1000DPS: f32 = 32.8;   // LSB/(°/s)
pub const GYRO_SCALE_FACTOR_2000DPS: f32 = 16.4;   // LSB/(°/s)

/// Tasa base del ICM20948 (Hz)
pub const ICM20948_BASE_SAMPLE_RATE: f32 = 1125.0;

/// Estructura para datos de aceleración en unidades físicas (g)
#[derive(Debug, Clone, Copy)]
pub struct AccelData {
    /// Aceleración en el eje X (g)
    pub x: f32,
    /// Aceleración en el eje Y (g)
    pub y: f32,
    /// Aceleración en el eje Z (g)
    pub z: f32,
    /// Marca de tiempo en microsegundos
    pub timestamp_us: u64,
}

/// Estructura para datos del giroscopio en unidades físicas (grados/segundo)
#[derive(Debug, Clone, Copy)]
pub struct GyroData {
    /// Velocidad angular en el eje X (°/s)
    pub x: f32,
    /// Velocidad angular en el eje Y (°/s)
    pub y: f32,
    /// Velocidad angular en el eje Z (°/s)
    pub z: f32,
    /// Marca de tiempo en microsegundos
    pub timestamp_us: u64,
}

/// Configuración de la tasa de muestreo (ODR)
#[derive(Debug, Clone, Copy)]
pub enum SampleRate {
    /// 1125Hz (tasa base, divider = 0)
    Hz1125,
    /// 562.5Hz (divider = 1)
    Hz562_5,
    /// 375Hz (divider = 2)
    Hz375,
    /// 281.25Hz (divider = 3)
    Hz281_25,
    /// 225Hz (divider = 4)
    Hz225,
    /// 187.5Hz (divider = 5)
    Hz187_5,
    /// 112.5Hz (divider = 9)
    Hz112_5,
    /// 56.25Hz (divider = 19) - Recomendada para batching
    Hz56_25,
    /// 28.125Hz (divider = 39)
    Hz28_125,
    /// Tasa personalizada (especificar divider)
    Custom(u8),
}

impl SampleRate {
    /// Convierte la enumeración a un valor de divider
    pub fn to_divider(&self) -> u8 {
        match self {
            SampleRate::Hz1125 => 0,
            SampleRate::Hz562_5 => 1,
            SampleRate::Hz375 => 2,
            SampleRate::Hz281_25 => 3,
            SampleRate::Hz225 => 4,
            SampleRate::Hz187_5 => 5,
            SampleRate::Hz112_5 => 9,
            SampleRate::Hz56_25 => 19,
            SampleRate::Hz28_125 => 39,
            SampleRate::Custom(div) => *div,
        }
    }
    
    /// Calcula la frecuencia real en Hz
    pub fn to_frequency(&self) -> f32 {
        ICM20948_BASE_SAMPLE_RATE / (1.0 + self.to_divider() as f32)
    }
}

/// Configuración del filtro paso bajo para el acelerómetro
/// bits[0] = FCHOICE, bits[2:1] = FSCALE, bits[5:3] = DLFCFG
pub enum AccelLpfSetting {
    /// OFF(0)
    LpOFF = 0b00000000,
    /// 246 Hz
    Lp246_0Hz = 0b00000001,
    /// 246 Hz (2)
    Lp246_0Hz2 = 0b00001001,
    /// 111 Hz    
    Lp111_4Hz = 0b00010001,
    /// 50 Hz
    Lp50_4Hz = 0b00011001,
    /// 24 Hz
    Lp23_9Hz = 0b00100001,
    /// 12 Hz
    Lp11_5Hz = 0b00101001,
    /// 6 Hz
    Lp5_7Hz = 0b00110001,
    /// 473 Hz
    Lp473_0Hz = 0b00111001,
}

/// Configuración del filtro paso bajo para el giroscopio
/// bits[0] = FCHOICE, bits[2:1] = FSCALE, bits[5:3] = DLFCFG
pub enum GyroLpfSetting {
    /// OFF(0)
    LpOFF = 0b00000000,
    /// 196.6 Hz (0)
    Lp196_6Hz = 0b00000001,
    /// 151.8 Hz
    Lp151_8Hz = 0b00001001,
    /// 119.5 Hz
    Lp119_5Hz = 0b00010001,
    /// 51.2 Hz
    Lp51_2Hz = 0b00011001,
    /// 23.9 Hz
    Lp23_9Hz = 0b00100001,
    /// 11.6 Hz
    Lp11_6Hz = 0b00101001,
    /// 5.7 Hz
    Lp5_7Hz = 0b00110001,
    /// 176 Hz
    Lp361_4z = 0b00111001,
}

/// Implementaciones de control básico para ICM20948
impl<I, D, E> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
{
    /// Activa o desactiva el acelerómetro
    pub fn enable_accelerometer(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let mut sensors_enabled = 0;
        
        // Si habilitamos el acelerómetro, añadimos su bit a la máscara
        if enable {
            sensors_enabled |= ACCEL_AVAILABLE;
        }
        
        // Verificar si el giroscopio está activado actualmente
        if self.is_gyro_enabled() {
            sensors_enabled |= GYRO_AVAILABLE;
        }
        
        // Verificar si el compás está activado actualmente (si está disponible)
        if self.base_state.compass_config.is_some() {
            sensors_enabled |= SECONDARY_COMPASS_AVAILABLE;
        }
        
        // Aplica la configuración de sensores
        self.enable_hw_sensors(sensors_enabled)
    }
    
    /// Activa o desactiva el giroscopio
    pub fn enable_gyroscope(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let mut sensors_enabled = 0;
        
        // // Verificar si el acelerómetro está activado actualmente
        // if self.inv_androidSensorsOn_mask[0] & crate::device::INV_NEEDS_ACCEL_MASK != 0 {
        //     sensors_enabled |= ACCEL_AVAILABLE;
        // }
        
        // Si habilitamos el giroscopio, añadimos su bit a la máscara
        if enable {
            sensors_enabled |= GYRO_AVAILABLE;
        }
        
        // Verificar si el compás está activado actualmente (si está disponible)
        if self.base_state.compass_config.is_some() {
            sensors_enabled |= SECONDARY_COMPASS_AVAILABLE;
        }
        
        // Aplica la configuración de sensores
        self.enable_hw_sensors(sensors_enabled)
    }
    
    /// Configura la tasa de muestreo del acelerómetro
    pub fn set_accelerometer_sample_rate(&mut self, rate: SampleRate) -> Result<(), Icm20948Error> {
        self.set_accel_divider(rate.to_divider() as u16)
    }
    
    /// Configura la tasa de muestreo del giroscopio
    pub fn set_gyroscope_sample_rate(&mut self, rate: SampleRate) -> Result<(), Icm20948Error> {
        self.set_gyro_divider(rate.to_divider())
    }
    
    /// Obtiene la tasa de muestreo actual del acelerómetro en Hz
    pub fn get_accelerometer_sample_rate(&self) -> f32 {
        ICM20948_BASE_SAMPLE_RATE / (1.0 + self.get_accel_divider() as f32)
    }
    
    /// Obtiene la tasa de muestreo actual del giroscopio en Hz
    pub fn get_gyroscope_sample_rate(&self) -> f32 {
        ICM20948_BASE_SAMPLE_RATE / (1.0 + self.get_gyro_divider() as f32)
    }
    
    /// Configura la interrupción de datos listos (data ready)
    pub fn configure_data_ready_interrupt(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        // Leer configuración actual
        let mut int_enable = self.read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)?;
        
        // Modificar bit de data ready (bit 0)
        if enable {
            int_enable |= 0x01;  // Set bit 0
        } else {
            int_enable &= !0x01; // Clear bit 0
        }
        
        // Escribir nueva configuración
        self.write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE, int_enable)?;
        
        Ok(())
    }
    
    /// Configura el modo de bajo consumo o alto rendimiento
    pub fn set_low_power_mode(&mut self, low_power: bool) -> Result<(), Icm20948Error> {
        self.set_lowpower_or_highperformance(if low_power { 1 } else { 0 })
    }
    
    /// Activa el FIFO
    pub fn enable_fifo(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let mut int_enable = self.read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)?;

        if enable {
            int_enable |= 0x20; // bits::BIT_DATA_RDY_EN;
        } else {
            int_enable &= !0x20; // !bits::BIT_DATA_RDY_EN;
        }

        self.write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE, int_enable)?;
        
        Ok(())
    }
    
    /// Lee la cuenta actual del FIFO
    pub fn read_fifo_count(&mut self) -> Result<u16, Icm20948Error> {
        // Leer los bytes del contador FIFO
        let mut fifo_count = [0u8; 2];
        self.read_mems_regs::<bank0::Bank>(bank0::FIFO_COUNTH, &mut fifo_count[0..1])?;
        self.read_mems_regs::<bank0::Bank>(bank0::FIFO_COUNTL, &mut fifo_count[1..2])?;
        
        // Combinar bytes para obtener el contador de 16 bits
        let count = ((fifo_count[0] as u16) << 8) | fifo_count[1] as u16;
        
        Ok(count)
    }
    
    /// Lee datos del FIFO
    pub fn read_fifo_data(&mut self, buffer: &mut [u8], count: u16) -> Result<(), Icm20948Error> {
        if buffer.len() < count as usize {
            return Err(Icm20948Error::InvalidParameter);
        }
        
        // Leer datos del FIFO
        for i in 0..count as usize {
            self.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut buffer[i..i+1])?;
        }
        
        Ok(())
    }
    
    /// Configura qué datos se envían al FIFO
    pub fn configure_fifo_data(&mut self, accel: bool, gyro: bool, temp: bool) -> Result<(), Icm20948Error> {
        // Configurar FIFO_EN_1 (accel y temperatura)
        let mut fifo_en_1 = 0u8;
        if accel {
            fifo_en_1 |= 0x08; // Bit 3: ACCEL_FIFO_EN
        }
        if temp {
            fifo_en_1 |= 0x80; // Bit 7: TEMP_FIFO_EN
        }
        self.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_1, fifo_en_1)
            .map_err(|_| Icm20948Error::InterfaceError)?;
        
        // Configurar FIFO_EN_2 (giroscopio)
        let mut fifo_en_2 = 0u8;
        if gyro {
            fifo_en_2 |= 0x80; // Bit 7: GYRO_X_FIFO_EN
            fifo_en_2 |= 0x40; // Bit 6: GYRO_Y_FIFO_EN
            fifo_en_2 |= 0x20; // Bit 5: GYRO_Z_FIFO_EN
        }
        self.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_2, fifo_en_2)
            .map_err(|_| Icm20948Error::InterfaceError)?;
        
        Ok(())
    }

    /// Set LPF for accelerometer
    pub fn set_accel_lpf(&mut self, setting: AccelLpfSetting) -> Result<(), Icm20948Error> {
        self.modify_mems_reg::<bank2::Bank, _>(bank2::ACCEL_CONFIG_1, |val| val & !bits::LPF_SETTING | setting as u8)
    }

    /// Set LPF for gyroscope
    pub fn set_gyro_lpf(&mut self, setting: GyroLpfSetting) -> Result<(), Icm20948Error> {
        self.modify_mems_reg::<bank2::Bank, _>(bank2::GYRO_CONFIG_1, |val| val & !bits::LPF_SETTING | setting as u8)
    }
}