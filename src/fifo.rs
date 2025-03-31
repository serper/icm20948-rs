//! Implementación para manejar el FIFO del ICM20948
//!
//! Este módulo proporciona funcionalidades para configurar, leer
//! y procesar los datos del buffer FIFO del ICM20948.

use crate::device::{Icm20948, Icm20948Error};
use crate::register::registers::bank0;
use crate::controls::{AccelData, GyroData};
use crate::interface::Interface;
use crate::base::{SystemTimeSource, TimeSource};
use embedded_hal::blocking::delay::DelayMs;

/// Error específico para operaciones del FIFO
#[derive(Debug)]
pub enum FifoError {
    /// Error de dispositivo subyacente
    DeviceError(Icm20948Error),
    /// Datos insuficientes en el FIFO
    InsufficientData,
    /// Desbordamiento del FIFO
    FifoOverflow,
    /// Error al procesar los datos
    ProcessingError,
    /// Error en la configuración
    ConfigError,
}

impl From<Icm20948Error> for FifoError {
    fn from(err: Icm20948Error) -> Self {
        FifoError::DeviceError(err)
    }
}

/// Configuración del FIFO
#[derive(Debug, Clone)]
pub struct FifoConfig {
    /// Habilitar datos de acelerómetro en FIFO
    pub accel: bool,
    /// Habilitar datos de giroscopio en FIFO
    pub gyro: bool,
    /// Habilitar datos de temperatura en FIFO
    pub temp: bool,
    /// Habilitar el vaciado automático cuando se llena
    pub auto_flush: bool,
    /// Umbral de llenado para generar interrupción (bytes)
    pub watermark: u16,
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self {
            accel: true,
            gyro: true,
            temp: false,
            auto_flush: true,
            watermark: 1024,
        }
    }
}

/// Estado del FIFO
#[derive(Debug, Clone)]
pub struct FifoState {
    /// Tipo de datos esperados en el FIFO
    pub config: FifoConfig,
    /// Buffer para almacenar temporalmente los datos del FIFO
    pub buffer: Vec<u8>,
    /// Contador de paquetes procesados
    pub packets_processed: u32,
    /// Último timestamp de lectura
    pub last_timestamp_us: u64,
}

impl Default for FifoState {
    fn default() -> Self {
        Self {
            config: FifoConfig::default(),
            buffer: Vec::with_capacity(4096),
            packets_processed: 0,
            last_timestamp_us: 0,
        }
    }
}

/// Datos decodificados de un paquete FIFO
#[derive(Debug, Clone)]
pub struct FifoDecodedData {
    /// Datos de acelerómetro (si están habilitados)
    pub accel: Option<AccelData>,
    /// Datos de giroscopio (si están habilitados)
    pub gyro: Option<GyroData>,
    /// Datos de temperatura (si está habilitada)
    pub temperature: Option<f32>,
    /// Timestamp del paquete
    pub timestamp_us: u64,
}

impl Default for FifoDecodedData {
    fn default() -> Self {
        Self {
            accel: None,
            gyro: None,
            temperature: None,
            timestamp_us: 0,
        }
    }
}

// Tamaños de datos en el FIFO
const FIFO_HEADER_SIZE: usize = 1;
const FIFO_ACCEL_DATA_SIZE: usize = 6;
const FIFO_GYRO_DATA_SIZE: usize = 6;
const FIFO_TEMP_DATA_SIZE: usize = 2;

// Bits de la cabecera FIFO
const FIFO_HEADER_ACCEL: u8 = 0x01;
const FIFO_HEADER_GYRO: u8 = 0x02;
const FIFO_HEADER_TEMP: u8 = 0x04;
const FIFO_HEADER_COMPASS: u8 = 0x08;

/// Implementación de las operaciones FIFO para ICM20948
impl<I, D, E> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
{
    /// Configura el FIFO según las especificaciones proporcionadas
    pub fn configure_fifo(&mut self, config: &FifoConfig) -> Result<(), Icm20948Error> {
        // Resetear el FIFO
        self.reset_fifo()?;
        
        // Configurar FIFO_EN_1 (accel y temperatura)
        let mut fifo_en_1: u8 = 0;
        if config.accel {
            fifo_en_1 |= 0x08; // Bit 3: ACCEL_FIFO_EN
        }
        if config.temp {
            fifo_en_1 |= 0x80; // Bit 7: TEMP_FIFO_EN
        }
        
        // Configurar FIFO_EN_2 (giroscopio)
        let mut fifo_en_2: u8 = 0;
        if config.gyro {
            fifo_en_2 |= 0x80; // Bit 7: GYRO_X_FIFO_EN
            fifo_en_2 |= 0x40; // Bit 6: GYRO_Y_FIFO_EN
            fifo_en_2 |= 0x20; // Bit 5: GYRO_Z_FIFO_EN
        }
        
        // Establecer configuración FIFO
        self.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_1, fifo_en_1)?;
        self.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_2, fifo_en_2)?;
        
        // Configurar umbral de interrupción FIFO si es necesario
        // En futuras implementaciones
        
        // Configurar MODE
        let mode = if config.auto_flush { 0x01 } else { 0x00 };
        self.write_mems_reg::<bank0::Bank>(bank0::FIFO_MODE, mode)?;
        
        // Habilitar FIFO
        let mut user_ctrl = self.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)?;
        user_ctrl |= 0x40; // FIFO_EN bit
        self.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL,user_ctrl)?;
        
        Ok(())
    }
    
    /// Resetea el FIFO
    pub fn reset_fifo(&mut self) -> Result<(), Icm20948Error> {
        // Leer registro USER_CTRL actual
        let mut user_ctrl = self.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)?;
        
        // Establecer bit de reset FIFO
        user_ctrl |= 0x04; // FIFO_RST bit
        self.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL, user_ctrl)?;
        
        // Esperar un poco
        self.delay.delay_ms(1);
        
        // Limpiar bit de reset
        user_ctrl &= !0x04;
        self.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL, user_ctrl)?;
        
        Ok(())
    }
    
    /// Lee el contador de bytes disponibles en el FIFO
    pub fn get_fifo_count(&mut self) -> Result<u16, Icm20948Error> {
        // Leer contador FIFO (2 bytes)
        let fifo_count_h = self.read_mems_reg::<bank0::Bank>(bank0::FIFO_COUNTH)?;
        let fifo_count_l = self.read_mems_reg::<bank0::Bank>(bank0::FIFO_COUNTL)?;
        
        // Combinar bytes
        let count = ((fifo_count_h as u16) << 8) | fifo_count_l as u16;
        
        Ok(count)
    }
    
    /// Lee datos del FIFO
    pub fn read_fifo_bytes(&mut self, buffer: &mut [u8], count: usize) -> Result<(), Icm20948Error> {
        if count == 0 || buffer.len() < count {
            return Err(Icm20948Error::InvalidParameter);
        }

        let mut tmp = vec![0u8; count];
        // Leer datos del FIFO
        self.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut tmp)?;
        buffer.copy_from_slice(&tmp[..count]);

        Ok(())
    }

    /// Procesa los datos del FIFO y los decodifica según la configuración
    pub fn process_fifo_data(&mut self, state: &mut FifoState, 
                            callback: &mut dyn FnMut(&FifoDecodedData) -> Result<(), FifoError>) 
                            -> Result<u16, FifoError> {
        // Verificar cuántos bytes hay disponibles
        let fifo_count = self.get_fifo_count()?;
        
        if fifo_count == 0 {
            return Ok(0); // No hay datos disponibles
        }
        
        // Si hay desbordamiento
        if fifo_count >= 4096 {
            self.reset_fifo()?;
            return Err(FifoError::FifoOverflow);
        }
        
        // Asegurar que el buffer tiene capacidad suficiente
        if state.buffer.capacity() < fifo_count as usize {
            state.buffer.reserve(fifo_count as usize - state.buffer.capacity());
        }
        
        // Limpiar buffer y ajustar tamaño
        state.buffer.clear();
        state.buffer.resize(fifo_count as usize, 0);
        
        // Leer todos los datos disponibles
        self.read_fifo_bytes(&mut state.buffer, fifo_count as usize)?;
        
        // Procesar los datos
        let mut processed_bytes = 0;

        // Calcular el timestamps de la primera muestra
        let accel_sample_rate = self.get_accelerometer_sample_rate() as f32;
        let gyro_sample_rate = self.get_gyroscope_sample_rate() as f32;
        let accel_scale_factor = 1.0 / accel_sample_rate;
        let gyro_scale_factor = 1.0 / gyro_sample_rate;
        let timestamp_now = SystemTimeSource.get_timestamp_us();
        let mut timestamp_now_accel= timestamp_now - (fifo_count as f32 * accel_scale_factor) as u64;
        let mut timestamp_now_gyro = timestamp_now - (fifo_count as f32 * gyro_scale_factor) as u64;
        
        while processed_bytes < fifo_count as usize {
            // Verificar que tenemos al menos la cabecera
            if processed_bytes + FIFO_HEADER_SIZE > fifo_count as usize {
                break;
            }
            
            let header = state.buffer[processed_bytes];
            processed_bytes += FIFO_HEADER_SIZE;
            
            // Preparar estructura de datos decodificados
            let mut decoded_data = FifoDecodedData {
                accel: None,
                gyro: None,
                temperature: None,
                timestamp_us: timestamp_now,
            };
            
            // Decodificar acelerómetro
            if (header & FIFO_HEADER_ACCEL) != 0 {
                if processed_bytes + FIFO_ACCEL_DATA_SIZE > fifo_count as usize {
                    return Err(FifoError::InsufficientData);
                }
                
                let accel_data = &state.buffer[processed_bytes..processed_bytes + FIFO_ACCEL_DATA_SIZE];
                processed_bytes += FIFO_ACCEL_DATA_SIZE;
                
                // Convertir bytes a valores
                let accel_x = ((accel_data[0] as i16) << 8) | accel_data[1] as i16;
                let accel_y = ((accel_data[2] as i16) << 8) | accel_data[3] as i16;
                let accel_z = ((accel_data[4] as i16) << 8) | accel_data[5] as i16;
                
                // Aplicar conversión según escala configurada
                let accel = AccelData {
                    x: self.convert_accel_to_g(accel_x),
                    y: self.convert_accel_to_g(accel_y),
                    z: self.convert_accel_to_g(accel_z),
                    timestamp_us: timestamp_now_accel,
                };
                timestamp_now_accel += accel_scale_factor as u64;
                decoded_data.accel = Some(accel);
            }
            
            // Decodificar giroscopio
            if (header & FIFO_HEADER_GYRO) != 0 {
                if processed_bytes + FIFO_GYRO_DATA_SIZE > fifo_count as usize {
                    return Err(FifoError::InsufficientData);
                }
                
                let gyro_data = &state.buffer[processed_bytes..processed_bytes + FIFO_GYRO_DATA_SIZE];
                processed_bytes += FIFO_GYRO_DATA_SIZE;
                
                // Convertir bytes a valores
                let gyro_x = ((gyro_data[0] as i16) << 8) | gyro_data[1] as i16;
                let gyro_y = ((gyro_data[2] as i16) << 8) | gyro_data[3] as i16;
                let gyro_z = ((gyro_data[4] as i16) << 8) | gyro_data[5] as i16;
                
                // Aplicar conversión según escala configurada
                let gyro = GyroData {
                    x: self.convert_gyro_to_dps(gyro_x),
                    y: self.convert_gyro_to_dps(gyro_y),
                    z: self.convert_gyro_to_dps(gyro_z),
                    timestamp_us: timestamp_now_gyro,
                };
                
                timestamp_now_gyro += gyro_scale_factor as u64;
                decoded_data.gyro = Some(gyro);
            }
            
            // Decodificar temperatura
            if (header & FIFO_HEADER_TEMP) != 0 {
                if processed_bytes + FIFO_TEMP_DATA_SIZE > fifo_count as usize {
                    return Err(FifoError::InsufficientData);
                }
                
                let temp_data = &state.buffer[processed_bytes..processed_bytes + FIFO_TEMP_DATA_SIZE];
                processed_bytes += FIFO_TEMP_DATA_SIZE;
                
                // Convertir bytes a valor
                let temp_raw = ((temp_data[0] as i16) << 8) | temp_data[1] as i16;
                
                // Convertir a temperatura en °C
                let temp_c = temp_raw as f32 / 333.87 + 21.0;
                decoded_data.temperature = Some(temp_c);
            }
            
            // Ignorar datos de magnetómetro por ahora
            if (header & FIFO_HEADER_COMPASS) != 0 {
                // En una implementación futura
            }
            
            // Llamar al callback con los datos decodificados
            callback(&decoded_data)?;
            
            // Actualizar contador de paquetes
            state.packets_processed += 1;
        }
        
        // Actualizar timestamp
        state.last_timestamp_us = timestamp_now;
        
        Ok(processed_bytes as u16)
    }
}
