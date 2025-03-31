//! Implementación para manejar el FIFO del DMP en ICM20948
use crate::base::{SystemTimeSource, TimeSource};
use crate::device::{Icm20948, Icm20948Error};
use crate::register::{registers::bank0, dmp};
use crate::types::bits;
use crate::interface::Interface;
use embedded_hal::blocking::delay::DelayMs;
use core::fmt::Debug;
// extern crate alloc;
// use alloc::vec::Vec;

/// Error específico para operaciones del DMP FIFO
#[derive(Debug, Clone)]
pub enum DmpFifoError {
    /// Error de dispositivo subyacente
    DeviceError(Icm20948Error),
    /// Datos insuficientes en el FIFO
    InsufficientData,
    /// Desbordamiento del FIFO
    FifoOverflow,
    /// Paquete DMP inválido
    InvalidPacket,
    /// Error al procesar los datos
    ProcessingError,
    /// Error en la configuración
    ConfigError,
    /// No hay datos disponibles
    NoDataAvailable,
}

impl From<Icm20948Error> for DmpFifoError {
    fn from(err: Icm20948Error) -> Self {
        DmpFifoError::DeviceError(err)
    }
}

/// Configuración del FIFO del DMP
#[derive(Debug, Clone)]
pub struct DmpFifoConfig {
    /// Habilitar datos de acelerómetro en FIFO
    pub accel_enable: bool,
    /// Habilitar datos de giroscopio en FIFO
    pub gyro_enable: bool,
    /// Habilitar datos de cuaternión de 6 ejes (Game Rotation Vector)
    pub quat6_enable: bool,
    /// Habilitar datos de cuaternión de 9 ejes (Rotation Vector)
    pub quat9_enable: bool,
    /// Umbral de llenado para generar interrupción (bytes)
    pub fifo_watermark: u16,
}

impl Default for DmpFifoConfig {
    fn default() -> Self {
        Self {
            accel_enable: true,
            gyro_enable: true,
            quat6_enable: true,
            quat9_enable: false,
            fifo_watermark: 1024,
        }
    }
}

/// Estado del FIFO del DMP
#[derive(Debug, Clone)]
pub struct DmpFifoState {
    /// Tipo de datos esperados en el FIFO
    pub config: DmpFifoConfig,
    /// Buffer para almacenar temporalmente los datos del FIFO
    pub buffer: Vec<u8>,
    /// Datos de cuaternión de 6 ejes
    pub quaternion6: Option<QuaternionData>,
    /// Datos de cuaternión de 9 ejes
    pub quaternion9: Option<QuaternionData>,
    /// Datos de acelerómetro
    pub accel_data: Option<[i16; 3]>,
    /// Datos de giroscopio
    pub gyro_data: Option<[i16; 3]>,
    /// Contador de paquetes procesados
    pub packets_processed: u32,
    /// Último timestamp de lectura
    pub last_timestamp_us: u64,
}

impl Default for DmpFifoState {
    fn default() -> Self {
        Self {
            config: DmpFifoConfig::default(),
            buffer: Vec::with_capacity(4096),
            quaternion6: None,
            quaternion9: None,
            accel_data: None,
            gyro_data: None,
            packets_processed: 0,
            last_timestamp_us: 0,
        }
    }
}

/// Datos de cuaternión (vector de rotación)
#[derive(Debug, Clone, Copy)]
pub struct QuaternionData {
    /// Componente W (escalar) del cuaternión
    pub w: f32,
    /// Componente X del cuaternión
    pub x: f32,
    /// Componente Y del cuaternión
    pub z: f32,
    /// Componente Z del cuaternión
    pub y: f32,
    /// Precisión estimada en radianes
    pub accuracy_rad: f32,
    /// Timestamp de la muestra
    pub timestamp_us: u64,
}

impl Default for QuaternionData {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            accuracy_rad: 0.0,
            timestamp_us: 0,
        }
    }
}

/// Datos de orientación (ángulos de Euler)
#[derive(Debug, Clone, Copy)]
pub struct OrientationData {
    /// Ángulo de cabeceo (pitch) en grados
    pub pitch: f32,
    /// Ángulo de balanceo (roll) en grados
    pub roll: f32,
    /// Ángulo de guiñada (yaw) en grados
    pub yaw: f32,
    /// Precisión estimada de la orientación en grados
    pub heading_accuracy: f32,
    /// Timestamp de la muestra
    pub timestamp_us: u64,
}

impl Default for OrientationData {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            heading_accuracy: 0.0,
            timestamp_us: 0,
        }
    }
}

// Constantes para los tipos de paquete DMP
const PACKET_QUAT9: u8 = 0x4D;
const PACKET_QUAT6: u8 = 0x42;
const PACKET_ACCEL: u8 = 0x4C;
const PACKET_GYRO: u8 = 0x4B;

/// Implementación de funciones para el FIFO del DMP
impl<I, D, E> Icm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    // E: Debug,
    // Icm20948Error: From<E>,
{
    /// Configura el FIFO del DMP según las especificaciones proporcionadas
    pub fn configure_dmp_fifo(&mut self, config: &DmpFifoConfig) -> Result<(), Icm20948Error> {
        // Asegurarse de que el DMP está habilitado
        if !self.is_firmware_loaded() {
            return Err(Icm20948Error::InvalidOperation);
        }
        
        // Resetear el FIFO
        self.reset_fifo()?;
        
        // Configurar las máscaras de salida de datos DMP
        let mut data_out_ctl1: u16 = 0;
        
        if config.accel_enable {
            data_out_ctl1 |= crate::dmp::output_mask::ACCEL_SET;
        }
        
        if config.gyro_enable {
            data_out_ctl1 |= crate::dmp::output_mask::GYRO_SET;
        }
        
        if config.quat6_enable {
            data_out_ctl1 |= crate::dmp::output_mask::QUAT6_SET;
        }
        
        if config.quat9_enable {
            data_out_ctl1 |= crate::dmp::output_mask::QUAT9_SET;
        }
        
        // Escribir configuración al DMP
        let bytes = data_out_ctl1.to_be_bytes();
        self.write_mems(dmp::DATA_OUT_CTL1, &bytes)?;
        
        // Configurar umbral de FIFO
        let bytes = config.fifo_watermark.to_be_bytes();
        self.write_mems(dmp::FIFO_WATERMARK, &bytes)?;
        
        // Configurar interrupción DMP
        self.set_int1_assertion(1)?;
        
        // Habilitar FIFO
        self.configure_fifo_dmp_enable(true)?;
        
        Ok(())
    }
    
    /// Habilita o deshabilita el FIFO del DMP
    fn configure_fifo_dmp_enable(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        // Leer registro USER_CTRL
        let mut user_ctrl = self.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL).map_err(|_| Icm20948Error::InvalidOperation)?;
        
        if enable {
            // Habilitar DMP y FIFO
            user_ctrl |= bits::DMP_EN | bits::FIFO_EN;
        } else {
            // Deshabilitar DMP y FIFO
            user_ctrl &= !(bits::DMP_EN | bits::FIFO_EN);
        }
        
        // Escribir configuración actualizada
        self.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL, user_ctrl).map_err(|_| Icm20948Error::InvalidOperation)?;

        Ok(())
    }
    
    /// Lee los datos del FIFO del DMP y los decodifica
    pub fn read_dmp_fifo_data(&mut self, state: &mut DmpFifoState) -> Result<(), DmpFifoError> {
        // Verificar si hay datos disponibles
        let fifo_count = self.get_fifo_count()?;
        
        if fifo_count == 0 {
            return Err(DmpFifoError::NoDataAvailable);
        }
        
        // Verificar desbordamiento
        if fifo_count >= 4096 {
            self.reset_fifo()?;
            return Err(DmpFifoError::FifoOverflow);
        }
        
        // Preparar buffer
        state.buffer.clear();
        state.buffer.resize(fifo_count as usize, 0);
        
        // Leer datos del FIFO
        self.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut state.buffer)?;
        
        // Procesar datos
        self.process_dmp_fifo_data(state)?;
        
        Ok(())
    }
    
    /// Procesa los datos del FIFO del DMP
    fn process_dmp_fifo_data(&mut self, state: &mut DmpFifoState) -> Result<(), DmpFifoError> {
        let mut index = 0;
        let timestamp_now = SystemTimeSource.get_timestamp_us();
        
        // Reiniciar datos anteriores
        state.accel_data = None;
        state.gyro_data = None;
        state.quaternion6 = None;
        state.quaternion9 = None;
        
        while index < state.buffer.len() {
            if index + 1 >= state.buffer.len() {
                break;  // No hay suficientes datos para el header
            }
            
            let header = state.buffer[index];
            index += 1;
            
            match header {
                PACKET_QUAT9 => {
                    // Procesar cuaternión de 9 ejes (w, x, y, z)
                    if index + 16 > state.buffer.len() {
                        return Err(DmpFifoError::InsufficientData);
                    }
                    
                    // Extraer los 4 valores de punto fijo (cada uno de 4 bytes)
                    let w = i32::from_be_bytes([
                        state.buffer[index], 
                        state.buffer[index+1], 
                        state.buffer[index+2],
                        state.buffer[index+3]
                    ]);
                    
                    let x = i32::from_be_bytes([
                        state.buffer[index+4], 
                        state.buffer[index+5],
                        state.buffer[index+6],
                        state.buffer[index+7]
                    ]);
                    
                    let y = i32::from_be_bytes([
                        state.buffer[index+8],
                        state.buffer[index+9],
                        state.buffer[index+10],
                        state.buffer[index+11]
                    ]);
                    
                    let z = i32::from_be_bytes([
                        state.buffer[index+12],
                        state.buffer[index+13],
                        state.buffer[index+14],
                        state.buffer[index+15]
                    ]);
                    
                    index += 16;
                    
                    // Convertir de punto fijo a flotante
                    // Los cuaterniones están normalizados, por lo que convertimos dividiendo por 2^30
                    const Q30: f32 = 1073741824.0; // 2^30
                    
                    state.quaternion9 = Some(QuaternionData {
                        w: w as f32 / Q30,
                        x: x as f32 / Q30,
                        y: y as f32 / Q30,
                        z: z as f32 / Q30,
                        accuracy_rad: 0.0,
                        timestamp_us: timestamp_now,
                    });
                },
                PACKET_QUAT6 => {
                    // Procesar cuaternión de 6 ejes (x, y, z)
                    if index + 12 > state.buffer.len() {
                        return Err(DmpFifoError::InsufficientData);
                    }
                    
                    // Extraer los 3 valores de punto fijo (cada uno de 4 bytes)
                    let x = i32::from_be_bytes([
                        state.buffer[index],
                        state.buffer[index+1],
                        state.buffer[index+2],
                        state.buffer[index+3]
                    ]);
                    
                    let y = i32::from_be_bytes([
                        state.buffer[index+4],
                        state.buffer[index+5],
                        state.buffer[index+6],
                        state.buffer[index+7]
                    ]);
                    
                    let z = i32::from_be_bytes([
                        state.buffer[index+8],
                        state.buffer[index+9],
                        state.buffer[index+10],
                        state.buffer[index+11]
                    ]);
                    
                    index += 12;
                    
                    // Convertir de punto fijo a flotante
                    const Q30: f32 = 1073741824.0; // 2^30
                    
                    state.quaternion6 = Some(QuaternionData {
                        w: 0.0,  // Para GRV, w se puede calcular a partir de x, y, z
                        x: x as f32 / Q30,
                        y: y as f32 / Q30,
                        z: z as f32 / Q30,
                        accuracy_rad: 0.0,
                        timestamp_us: timestamp_now,
                    });
                },
                PACKET_ACCEL => {
                    // Procesar datos de acelerómetro
                    if index + 6 > state.buffer.len() {
                        return Err(DmpFifoError::InsufficientData);
                    }
                    
                    // Extraer los 3 valores (cada uno de 2 bytes)
                    let x = i16::from_be_bytes([state.buffer[index], state.buffer[index+1]]);
                    let y = i16::from_be_bytes([state.buffer[index+2], state.buffer[index+3]]);
                    let z = i16::from_be_bytes([state.buffer[index+4], state.buffer[index+5]]);
                    
                    index += 6;
                    
                    state.accel_data = Some([x, y, z]);
                },
                PACKET_GYRO => {
                    // Procesar datos de giroscopio
                    if index + 6 > state.buffer.len() {
                        return Err(DmpFifoError::InsufficientData);
                    }
                    
                    // Extraer los 3 valores (cada uno de 2 bytes)
                    let x = i16::from_be_bytes([state.buffer[index], state.buffer[index+1]]);
                    let y = i16::from_be_bytes([state.buffer[index+2], state.buffer[index+3]]);
                    let z = i16::from_be_bytes([state.buffer[index+4], state.buffer[index+5]]);
                    
                    index += 6;
                    
                    state.gyro_data = Some([x, y, z]);
                },
                _ => {
                    // Tipo de paquete desconocido, intentar saltar al siguiente
                    return Err(DmpFifoError::InvalidPacket);
                }
            }
        }
        
        // Actualizar timestamp y contador de paquetes
        state.last_timestamp_us = timestamp_now;
        state.packets_processed += 1;
        
        Ok(())
    }
    
    /// Obtiene la orientación (ángulos de Euler) a partir de los datos del DMP
    pub fn get_dmp_orientation(&self, state: &DmpFifoState) -> Result<OrientationData, DmpFifoError> {
        // Intentar usar cuaternión de 9 ejes si está disponible
        if let Some(quat) = &state.quaternion9 {
            return Ok(self.quaternion_to_euler(quat));
        }
        
        // De lo contrario, usar cuaternión de 6 ejes
        if let Some(quat) = &state.quaternion6 {
            return Ok(self.quaternion_to_euler(quat));
        }
        
        Err(DmpFifoError::NoDataAvailable)
    }
    
    /// Convierte un cuaternión a ángulos de Euler (roll, pitch, yaw)
    fn quaternion_to_euler(&self, quat: &QuaternionData) -> OrientationData {
        // Reconstruir w para cuaternión de 6 ejes si es necesario
        let mut qw = quat.w;
        let qx = quat.x;
        let qy = quat.y;
        let qz = quat.z;
        
        if qw == 0.0 {
            // Recalcular w asumiendo que el cuaternión es unitario
            let squared_norm = qx*qx + qy*qy + qz*qz;
            if squared_norm < 1.0 {
                qw = (1.0 - squared_norm).sqrt();
            } else {
                qw = 0.0;  // Error, pero evitamos NaN
            }
        }
        
        // Calcular ángulos de Euler
        let roll = (2.0 * (qw * qx + qy * qz))
            .atan2(1.0 - 2.0 * (qx * qx + qy * qy))
            .to_degrees();
            
        let pitch = (2.0 * (qw * qy - qz * qx))
            .asin()
            .to_degrees();
            
        let yaw = (2.0 * (qw * qz + qx * qy))
            .atan2(1.0 - 2.0 * (qy * qy + qz * qz))
            .to_degrees();
        
        // Normalizar yaw a 0-360
        let yaw = if yaw < 0.0 { yaw + 360.0 } else { yaw };
        
        OrientationData {
            roll,
            pitch,
            yaw,
            heading_accuracy: quat.accuracy_rad.to_degrees(),
            timestamp_us: quat.timestamp_us,
        }
    }
    
    /// Obtiene el cuaternión de 6 ejes (Game Rotation Vector)
    pub fn get_dmp_quaternion6(&self, state: &DmpFifoState) -> Result<QuaternionData, DmpFifoError> {
        if let Some(quat) = &state.quaternion6 {
            Ok(*quat)
        } else {
            Err(DmpFifoError::NoDataAvailable)
        }
    }
    
    /// Obtiene el cuaternión de 9 ejes (Rotation Vector)
    pub fn get_dmp_quaternion9(&self, state: &DmpFifoState) -> Result<QuaternionData, DmpFifoError> {
        if let Some(quat) = &state.quaternion9 {
            Ok(*quat)
        } else {
            Err(DmpFifoError::NoDataAvailable)
        }
    }
    
    /// Configura la interrupción DMP
    fn set_int1_assertion(&mut self, enable: i32) -> Result<(), Icm20948Error> {
        // Leer registro INT_ENABLE
        let mut int_enable = self.read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)?;
        
        if enable != 0 {
            // Habilitar interrupción DMP
            int_enable |= bits::DMP_INT_EN;
        } else {
            // Deshabilitar interrupción DMP
            int_enable &= !bits::DMP_INT_EN;
        }
        
        // Escribir configuración actualizada
        self.write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE, int_enable)
    }
}
