//! Implementación para manejar el FIFO del DMP en ICM20948
use crate::base::{SystemTimeSource, TimeSource};
use crate::device::Icm20948Error;
use crate::dmp::DmpDriverIcm20948;
use crate::register::registers::bank0;
use crate::types::{bits, dmp_header, dmp_header2, dmp_packet_bytes, dmp_packet_bytes2};
use crate::interface::Interface;
use embedded_hal::blocking::delay::DelayMs;
use core::fmt::Debug;
// extern crate alloc;
// use alloc::vec::Vec;

use crate::conversion;

/// Datos de cuaternión procesados
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    /// Componente w (escalar)
    pub w: f32,
    /// Componente x
    pub x: f32,
    /// Componente y
    pub y: f32,
    /// Componente z
    pub z: f32,
    /// Precisión del rumbo en grados (solo para cuaternión de 9 ejes)
    pub heading_accuracy_deg: Option<f32>,
}

#[derive(Debug, Clone, Copy)]
pub struct ActRecognitionData {
    pub state_start: u8,            // Tipo de actividad (byte[0])
    pub state_end: u8,              // Tipo de actividad anterior (byte[1])
    pub timestamp: u32,             // Timestamp (byte[2..5])
}

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
    /// Error de lectura de datos
    DataError,
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
    /// Habilitar datos del compás en FIFO
    pub compass_enable: bool,
    /// Habilitar datos de cuaternión de 6 ejes (Game Rotation Vector)
    pub quat6_enable: bool,
    /// Habilitar datos de cuaternión de 9 ejes (Rotation Vector)
    pub quat9_enable: bool,
    /// Habilitar datos geomagnéticos en FIFO
    pub geomag_enable: bool,
    /// Habilitar detector de pasos
    pub step_detector_enable: bool,
    /// Habilitar sensor de presión
    pub pressure_enable: bool,
    /// Habilitar sensor de luz ambiental
    pub als_enable: bool,
    /// Umbral de llenado para generar interrupción (bytes)
    pub fifo_watermark: u16,
}

impl Default for DmpFifoConfig {
    fn default() -> Self {
        Self {
            accel_enable: true,
            gyro_enable: true,
            compass_enable: false,
            quat6_enable: false,
            quat9_enable: false,
            geomag_enable: false,
            step_detector_enable: false,
            pressure_enable: false,
            als_enable: false,
            fifo_watermark: 1024,
        }
    }
}

/// Estado del FIFO del DMP
#[derive(Debug, Clone)]
pub struct DmpFifoState {
    /// Tipo de datos esperados en el FIFO
    pub config: DmpFifoConfig,
    /// Header leído del FIFO
    pub header: u16,
    /// Header2 leído del FIFO
    pub header2: u16,
    /// Último header leído del FIFO, por si no habían datos disponibles
    pub last_header: u16,
    /// Último header2 leído del FIFO, por si no habían datos disponibles
    pub last_header2: u16,
    /// Datos de acelerómetro
    pub accel_data: Option<[i16; 3]>,
    /// Datos de giroscopio
    pub gyro_data: Option<[i16; 3]>,
    /// Gyro BIAS
    pub gyro_bias: Option<[i16; 3]>,
    /// Datos del compás (magnetómetro)
    pub compass_data: Option<[i16; 3]>,
    /// Datos de cuaternión de 6 ejes (Game Rotation Vector)
    pub quaternion6: Option<Quaternion>,
    /// Datos de cuaternión de 9 ejes (Rotation Vector)
    pub quaternion9: Option<Quaternion>,
    /// Datos de cuaternión de 6 ejes (pedometer)
    pub quaternionp6: Option<Quaternion>,
    /// Datos Geomagnéticos
    pub geomag_data: Option<Quaternion>,
    /// Datos de presión
    pub pressure_data: Option<[u8; 6]>,
    /// Calibración del giroscopio
    pub gyro_calibr: Option<Quaternion>,  
    /// Calibración del compás
    pub compass_calibr: Option<Quaternion>,
    /// Timestamp del pedómetro
    pub pedometer_timestamp: Option<u32>,
    /// Precisión del acelerómetro
    pub accel_accuracy: Option<u16>,
    /// Precisión del giroscopio
    pub gyro_accuracy: Option<u16>,
    /// Precisión del compás
    pub compass_accuracy: Option<u16>,
    /// FSync (synchronization) del FIFO
    pub fsync: Option<u16>,
    /// Estado de detección de pickup
    pub pickup_state: Option<bool>,
    /// Datos de actividad reconocida
    pub activity_recognition: Option<ActRecognitionData>,
    /// Secondary On Off
    pub secondary_on_off: Option<u16>,
    // footer
    pub footer: u16,
    /// Contador de paquetes procesados
    pub packets_processed: u32,
    /// Tamaño del paquete
    pub packet_size: usize,
    /// Paquete procesado
    pub packet: Vec<u8>,
    /// Count
    pub count: usize,
    /// Último timestamp de lectura
    pub last_timestamp_us: u64,
}

impl Default for DmpFifoState {
    fn default() -> Self {
        Self {
            config: DmpFifoConfig::default(),
            header: dmp_header::EMPTY,
            header2: dmp_header::EMPTY,
            last_header: 0xFFFF,
            last_header2: 0xFFFF,
            accel_data: None,
            gyro_data: None,
            gyro_bias: None,
            compass_data: None,
            quaternion6: None,
            quaternion9: None,
            quaternionp6: None,
            geomag_data: None,
            pressure_data: None,
            gyro_calibr: None,
            compass_calibr: None,
            pedometer_timestamp: None,
            accel_accuracy: None,
            gyro_accuracy: None,
            compass_accuracy: None,
            fsync: None,
            pickup_state: None,
            activity_recognition: None,
            secondary_on_off: None,
            footer: 0,
            packets_processed: 0,
            packet_size: 0,
            packet: Vec::new(),
            count: 0,
            last_timestamp_us: 0,
        }
    }
}

/// Extrae un valor i32 de 4 bytes en formato big endian
#[inline]
fn extract_i32_big_endian(buffer: &[u8], offset: usize) -> i32 {
    ((buffer[offset] as i32) << 24) |
    ((buffer[offset + 1] as i32) << 16) |
    ((buffer[offset + 2] as i32) << 8) |
    (buffer[offset + 3] as i32)
}

/// Extrae un valor i16 de 2 bytes en formato big endian
#[inline]
fn extract_i16_big_endian(buffer: &[u8], offset: usize) -> i16 {
    ((buffer[offset] as i16) << 8) | (buffer[offset + 1] as i16)
}

/// Procesa los datos del cuaternión de 6 ejes del FIFO
///
/// # Arguments
/// * `fifo_data` - Buffer con los datos del FIFO (debe tener al menos 12 bytes)
///
/// # Returns
/// Estructura Quaternion con los datos procesados
pub fn process_quaternion_6(fifo_data: &[u8]) -> Option<Quaternion> {
    if fifo_data.len() < dmp_packet_bytes::QUAT6 {
        return None;
    }

    // Extraer Q1, Q2, Q3 (en Big Endian)
    let q1 = extract_i32_big_endian(fifo_data, 0);
    let q2 = extract_i32_big_endian(fifo_data, 4);
    let q3 = extract_i32_big_endian(fifo_data, 8);
    
    // Convertir a valores flotantes aplicando el factor de escala de 2^30
    let q30 = 1_f32 / (1u32 << 30) as f32;
    let x = q1 as f32 * q30;
    let y = q2 as f32 * q30;
    let z = q3 as f32 * q30;
    
    // Calcular Q0 según la ecuación: Q0² + Q1² + Q2² + Q3² = 1
    // Por lo tanto, Q0 = sqrt(1 - (Q1² + Q2² + Q3²))
    let sum_squares = x*x + y*y + z*z;
    let w = if sum_squares < 1.0 {
        (1.0 - sum_squares).sqrt()
    } else {
        0.0 // En caso de drift que cause que sum_squares > 1
    };
    
    // Normalizar el cuaternión para corregir el drift
    let quaternion = normalize_quaternion(w, x, y, z);
    
    Some(Quaternion {
        w: quaternion.0,
        x: quaternion.1,
        y: quaternion.2,
        z: quaternion.3,
        heading_accuracy_deg: None,
    })
}

/// Procesa los datos del cuaternión de 6 ejes del pedometro del FIFO
///
/// # Arguments
/// * `fifo_data` - Buffer con los datos del FIFO (debe tener al menos 6 bytes)
///
/// # Returns
/// Estructura Quaternion con los datos procesados
pub fn process_quaternion_p6(fifo_data: &[u8]) -> Option<Quaternion> {
    if fifo_data.len() < dmp_packet_bytes::PQUAT6 {
        return None;
    }

    // Extraer Q1, Q2, Q3 (en Big Endian)
    let q1 = extract_i16_big_endian(fifo_data, 0);
    let q2 = extract_i16_big_endian(fifo_data, 2);
    let q3 = extract_i16_big_endian(fifo_data, 4);
    
    // Factor de escala correcto para valores de 16 bits (2^14)
    let q14 = 1_f32 / (1u32 << 14) as f32;
    let x = q1 as f32 * q14;
    let y = q2 as f32 * q14;
    let z = q3 as f32 * q14;
    
    // El resto del código es igual...
    let sum_squares = x*x + y*y + z*z;
    let w = if sum_squares < 1.0 {
        (1.0 - sum_squares).sqrt()
    } else {
        0.0 // En caso de drift que cause que sum_squares > 1
    };
    
    let quaternion = normalize_quaternion(w, x, y, z);
    
    Some(Quaternion {
        w: quaternion.0,
        x: quaternion.1,
        y: quaternion.2,
        z: quaternion.3,
        heading_accuracy_deg: None,
    })
}

/// Procesa los datos del cuaternión de 9 ejes del FIFO
///
/// # Arguments
/// * `fifo_data` - Buffer con los datos del FIFO (debe tener al menos 14 bytes)
///
/// # Returns
/// Estructura Quaternion con los datos procesados y la precisión del rumbo
pub fn process_quaternion_9(fifo_data: &[u8]) -> Option<Quaternion> {
    if fifo_data.len() < dmp_packet_bytes::QUAT9 {
        return None;
    }

    // Extraer Q1, Q2, Q3 (en Big Endian)
    let q1 = extract_i32_big_endian(fifo_data, 0);
    let q2 = extract_i32_big_endian(fifo_data, 4);
    let q3 = extract_i32_big_endian(fifo_data, 8);
    
    // Extraer la precisión del rumbo (heading accuracy)
    let heading_accuracy_raw = extract_i16_big_endian(fifo_data, 12);
    
    // Convertir a valores flotantes aplicando el factor de escala de 2^30
    let q30 = 1_f32 / (1u32 << 30) as f32;
    let x = q1 as f32 * q30;
    let y = q2 as f32 * q30;
    let z = q3 as f32 * q30;
    
    // Calcular Q0 según la ecuación: Q0² + Q1² + Q2² + Q3² = 1
    let sum_squares = x*x + y*y + z*z;
    let w = if sum_squares < 1.0 {
        (1.0 - sum_squares).sqrt()
    } else {
        0.0
    };
    
    // Normalizar el cuaternión para corregir el drift
    let quaternion = normalize_quaternion(w, x, y, z);
    
    // Convertir la precisión del rumbo a grados (escala típica: 1/100 grados)
    let heading_accuracy_deg = heading_accuracy_raw as f32 * 0.01;
    
    Some(Quaternion {
        w: quaternion.0,
        x: quaternion.1,
        y: quaternion.2,
        z: quaternion.3,
        heading_accuracy_deg: Some(heading_accuracy_deg),
    })
}

/// Normaliza un cuaternión para que su magnitud sea 1
///
/// # Arguments
/// * `w`, `x`, `y`, `z` - Componentes del cuaternión
///
/// # Returns
/// Cuaternión normalizado como tupla (w, x, y, z)
fn normalize_quaternion(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32, f32) {
    let magnitude = (w*w + x*x + y*y + z*z).sqrt();
    
    if magnitude > 1e-6 {  // Evitar división por cero
        let inv_magnitude = 1.0 / magnitude;
        (w * inv_magnitude, x * inv_magnitude, y * inv_magnitude, z * inv_magnitude)
    } else {
        (1.0, 0.0, 0.0, 0.0)  // Valor predeterminado en caso de magnitud casi cero
    }
}

/// Calcula los ángulos de Euler (roll, pitch, yaw) a partir de un cuaternión
///
/// # Arguments
/// * `quaternion` - Estructura con datos del cuaternión
///
/// # Returns
/// Ángulos de Euler en grados [roll, pitch, yaw]
pub fn quaternion_to_euler(quaternion: &Quaternion) -> [f32; 3] {
    let q0 = quaternion.w;
    let q1 = quaternion.x;
    let q2 = quaternion.y;
    let q3 = quaternion.z;
    
    // Calcular ángulos de Euler
    let roll = (2.0 * (q0 * q1 + q2 * q3)).atan2(1.0 - 2.0 * (q1 * q1 + q2 * q2));
    let pitch = (2.0 * (q0 * q2 - q3 * q1)).asin();
    let yaw = (2.0 * (q0 * q3 + q1 * q2)).atan2(1.0 - 2.0 * (q2 * q2 + q3 * q3));
    
    // Convertir a grados
    let rad_to_deg = 180.0 / std::f32::consts::PI;
    [
        roll * rad_to_deg,
        pitch * rad_to_deg,
        yaw * rad_to_deg
    ]
}

/// Procesa los datos de gravedad del FIFO
///
/// # Arguments
/// * `fifo_data` - Buffer con los datos de gravedad del FIFO (12 bytes)
///
/// # Returns
/// Vector de gravedad en m/s² [x, y, z]
pub fn process_gravity(fifo_data: &[u8]) -> Option<[f32; 3]> {
    if fifo_data.len() < 12 {
        return None;
    }
    
    let grav_x = extract_i32_big_endian(fifo_data, 0);
    let grav_y = extract_i32_big_endian(fifo_data, 4);
    let grav_z = extract_i32_big_endian(fifo_data, 8);
    
    Some(conversion::dmp_gravity_to_ms2([grav_x, grav_y, grav_z]))
}

/// Procesa los datos de aceleración lineal del FIFO
///
/// # Arguments
/// * `fifo_data` - Buffer con los datos de aceleración lineal del FIFO (12 bytes)
///
/// # Returns
/// Vector de aceleración lineal en m/s² [x, y, z]
pub fn process_linear_accel(fifo_data: &[u8]) -> Option<[f32; 3]> {
    if fifo_data.len() < 12 {
        return None;
    }
    
    let accel_x = extract_i32_big_endian(fifo_data, 0);
    let accel_y = extract_i32_big_endian(fifo_data, 4);
    let accel_z = extract_i32_big_endian(fifo_data, 8);
    
    Some(conversion::dmp_linear_accel_to_ms2([accel_x, accel_y, accel_z]))
}

/// Implementación de funciones para el FIFO del DMP
impl<I, D, E> DmpDriverIcm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    // E: Debug,
    // Icm20948Error: From<E>,
{
    // /// Configura el FIFO del DMP según las especificaciones proporcionadas
    // pub fn configure_dmp_fifo(&mut self, config: &DmpFifoConfig) -> Result<(), Icm20948Error> {
    //     // Asegurarse de que el DMP está habilitado
    //     if !self.is_firmware_loaded() {
    //         return Err(Icm20948Error::InvalidOperation);
    //     }

    //     // Resetear el FIFO
    //     self.device.reset_fifo()?;
        
    //     // Configurar las máscaras de salida de datos DMP
    //     let mut data_out_ctl1: u16 = 0;
        
    //     if config.accel_enable {
    //         data_out_ctl1 |= dmp::output_mask::ACCEL;
    //     }
        
    //     if config.gyro_enable {
    //         data_out_ctl1 |= dmp::output_mask::GYRO;
    //     }
        
    //     if config.compass_enable {
    //         data_out_ctl1 |= dmp::output_mask::CPASS;
    //     }
        
    //     if config.quat6_enable {
    //         data_out_ctl1 |= dmp::output_mask::QUAT6;
    //     }
        
    //     if config.quat9_enable {
    //         data_out_ctl1 |= dmp::output_mask::QUAT9;
    //     }
        
    //     if config.geomag_enable {
    //         data_out_ctl1 |= dmp::output_mask::GEOMAG;
    //     }
        
    //     if config.pressure_enable {
    //         data_out_ctl1 |= dmp::output_mask::PRESSURE;
    //     }
        
    //     if config.als_enable {
    //         data_out_ctl1 |= dmp::output_mask::ALS;
    //     }
        
    //     if config.step_detector_enable {
    //         data_out_ctl1 |= dmp::output_mask::PED_STEPDET;
    //     }
        
    //     // Escribir configuración al DMP
    //     let bytes = data_out_ctl1.to_be_bytes();
    //     self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL1, &bytes)?;
        
    //     // Configurar umbral de FIFO
    //     let bytes = config.fifo_watermark.to_be_bytes();
    //     self.device.write_mems(dmp::data_output_control::FIFO_WATERMARK, &bytes)?;
        
    //     // Configurar interrupción DMP
    //     self.set_int1_assertion(1)?;
        
    //     // Habilitar FIFO
    //     self.fifo_dmp_enable(true)?;
        
    //     Ok(())
    // }
    
    /// Reset el FIFO del DMP
    pub fn reset_fifo(&mut self) -> Result<(), Icm20948Error> {
        // Resetear el FIFO
        self.device.reset_fifo().map_err(|_| Icm20948Error::InvalidOperation)?;
        
        // Leer el registro FIFO_COUNT para verificar el estado
        let fifo_count = self.device.get_fifo_count()?;
        
        if fifo_count != 0 {
            return Err(Icm20948Error::InvalidOperation);
        }
        
        Ok(())
    }
    /// Habilita o deshabilita el FIFO del DMP
    pub fn dmp_fifo_enable(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        // Leer registro USER_CTRL
        self.device.enable_fifo(enable).map_err(|_| Icm20948Error::InvalidOperation)?;
        self.device.reset_fifo().map_err(|_| Icm20948Error::InvalidOperation)?;

        Ok(())
    }
    
    /// Lee los datos del FIFO del DMP y los decodifica
    pub fn read_dmp_fifo_data(&mut self, state: &mut DmpFifoState) -> Result<(), DmpFifoError> {
        // Verificar si hay datos disponibles
        let mut fifo_count = self.device.get_fifo_count()?;
        
        // Verificar desbordamiento
        if fifo_count >= 500 {
            self.device.reset_fifo()?;
            state.last_header = dmp_header::EMPTY;
            state.last_header2 = dmp_header::EMPTY;
            return Err(DmpFifoError::FifoOverflow);
        }
        
        if fifo_count < 2 {
            return Err(DmpFifoError::NoDataAvailable);
        }

        // Procesar datos
        // while fifo_count > 2 {                        
            // Procesar los datos leídos
            self.process_dmp_fifo_data(state, &mut fifo_count)?;
        // }
        // fifo_count = self.device.get_fifo_count()?;
        // // DEBUG: Leer el FIFO y mostrarlo por consola en HEX
        // let mut buffer = vec![0u8; fifo_count as usize];
        // self.device.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut buffer[..fifo_count])?;
        // println!("FIFO Data (count: {}): {:X?}", fifo_count, &buffer[..fifo_count]);

        Ok(())
    }
    
    /// Procesa los datos del FIFO del DMP
    fn process_dmp_fifo_data(&mut self, state: &mut DmpFifoState, count: &mut usize) -> Result<(), DmpFifoError> {
        let timestamp_now = SystemTimeSource.get_timestamp_us();
        
        if *count < 2 {
            return Err(DmpFifoError::InsufficientData);
        }
        
        if state.last_header == dmp_header::EMPTY {
            // Leer el header del FIFO
            let mut header = [0u8; 2];
            self.device.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut header)?;
            state.last_header = (header[0] as u16) << 8 | header[1] as u16;
            *count -= 2;
            state.last_header2 = dmp_header::EMPTY;
            state.packet_size = 2;
            // println!("Header: {:#X} cout: {}", state.last_header, *count);
        }

        if (state.last_header & dmp_header::HEADER2) != 0 {
            // Leer el header2 del FIFO si es necesario
            if state.last_header2 == dmp_header::EMPTY {
                if *count < dmp_packet_bytes::HEADER2 {
                    return Err(DmpFifoError::InsufficientData);
                }
                // Leer el header2 del FIFO
                let mut header2 = [0u8; 2];
                self.device.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut header2)?;
                state.last_header2 = u16::from_be_bytes(header2);
                *count -= 2;
                state.packet_size += 2;
                // println!("Header2: {:#X} cout: {}", state.last_header2, *count);
            }
        }

        // Calcular el tamaño del paquete
        let mut packet_size = 2; // Tamaño del footer
        for i in 0..16 {
            packet_size += match state.last_header & (1 << i) {
                dmp_header::ACCEL => dmp_packet_bytes::RAW_ACCEL,
                dmp_header::GYRO => dmp_packet_bytes::RAW_GYRO,
                dmp_header::COMPASS => dmp_packet_bytes::COMPASS,
                dmp_header::ALS => dmp_packet_bytes::ALS,
                dmp_header::QUAT6 => dmp_packet_bytes::QUAT6,
                dmp_header::QUAT9 => dmp_packet_bytes::QUAT9,
                dmp_header::PQUAT6 => dmp_packet_bytes::PQUAT6,
                dmp_header::GEOMAG => dmp_packet_bytes::GEOMAG,
                dmp_header::PRESSURE => dmp_packet_bytes::PRESSURE,
                dmp_header::GYRO_CALIBR => dmp_packet_bytes::GYRO_CALIBR,
                dmp_header::COMPASS_CALIBR => dmp_packet_bytes::COMPASS_CALIBR,
                dmp_header::STEP_DETECTOR => dmp_packet_bytes::STEP_DETECTOR,
                _ => 0,
            };
        }

        if state.last_header & dmp_header::HEADER2 != 0 {
            for i in 0..16 {
                packet_size += match state.last_header2 & (1 << i) {
                    dmp_header2::ACCEL_ACCURACY => dmp_packet_bytes2::ACCEL_ACCURACY,
                    dmp_header2::GYRO_ACCURACY => dmp_packet_bytes2::GYRO_ACCURACY,
                    dmp_header2::COMPASS_ACCURACY => dmp_packet_bytes2::COMPASS_ACCURACY,
                    dmp_header2::FSYNC => dmp_packet_bytes2::FSYNC,
                    dmp_header2::PICKUP => dmp_packet_bytes2::PICKUP,
                    dmp_header2::ACTIVITY_RECOG => dmp_packet_bytes2::ACTIVITY_RECOG,
                    dmp_header2::SECONDARY_ON_OFF => dmp_packet_bytes2::SECONDARY_ON_OFF,
                    _ => 0,
                };
            }
        }

        state.header = state.last_header;
        state.header2 = state.last_header2;
        state.packet_size += packet_size;

        if !state.packet.is_empty() {
            state.packet.clear();
        }
        
        state.packet.extend(state.header.to_be_bytes());
        if state.header2 != dmp_header::EMPTY {
            state.packet.extend(state.header2.to_be_bytes());
        }

        if *count < packet_size {
            return Err(DmpFifoError::InsufficientData);
        }

        // Leer el resto de los datos del FIFO
        let mut packet = vec![0u8; packet_size];
        self.device.read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut packet)?;
        state.packet.extend(packet.iter().cloned());
        *count -= packet_size;

        let mut idx = 0;
        // Procesamos el paquete            
        if state.last_header & dmp_header::ACCEL != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::RAW_ACCEL];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::RAW_ACCEL]);
            idx += dmp_packet_bytes::RAW_ACCEL;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            
            state.accel_data = Some([x, y, z]);
        }
        if state.last_header & dmp_header::GYRO != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::RAW_GYRO];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::RAW_GYRO]);
            idx += dmp_packet_bytes::RAW_GYRO;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            
            state.gyro_data = Some([x, y, z]);
        }
        if state.last_header & dmp_header::COMPASS != 0 {
            // Procesar datos del compás (magnetómetro)
            let mut buffer = [0u8; dmp_packet_bytes::COMPASS];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::COMPASS]);
            idx += dmp_packet_bytes::COMPASS;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            
            state.compass_data = Some([x, y, z]);
        }
        if state.last_header & dmp_header::ALS != 0 {
            // Procesar datos de proximidad
            if *count < dmp_packet_bytes::ALS {
                return Err(DmpFifoError::InsufficientData);
            }
            let mut buffer = [0u8; dmp_packet_bytes::ALS];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::ALS]);
            idx += dmp_packet_bytes::ALS;

            // Descartar datos ya que no está soportado por el sensor
        }
        if state.last_header & dmp_header::QUAT6 != 0 {
            // Procesar cuaternión de 6 ejes
            if *count < dmp_packet_bytes::QUAT6 {
                return Err(DmpFifoError::InsufficientData);
            }
            let mut buffer = [0u8; dmp_packet_bytes::QUAT6];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::QUAT6]);
            idx += dmp_packet_bytes::QUAT6;
            if let Some(quat) = process_quaternion_6(&buffer) {
                state.quaternion6 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::QUAT9 != 0 {
            // Procesar cuaternión de 9 ejes
            let mut buffer = [0u8; dmp_packet_bytes::QUAT9];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::QUAT9]);
            idx += dmp_packet_bytes::QUAT9;
            if let Some(quat) = process_quaternion_9(&buffer) {
                state.quaternion9 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::PQUAT6 != 0 {
            // Procesar cuaternión de 6 ejes del pedómetro
            let mut buffer = [0u8; dmp_packet_bytes::PQUAT6];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::PQUAT6]);
            idx += dmp_packet_bytes::PQUAT6;
            if let Some(quat) = process_quaternion_p6(&buffer) {
                state.quaternionp6 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::GEOMAG != 0 {
            // Procesar datos geomagnéticos (cuaternión geomagnético)
            let mut buffer = [0u8; dmp_packet_bytes::GEOMAG];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::GEOMAG]);
            idx += dmp_packet_bytes::GEOMAG;
            if let Some(quat) = process_quaternion_9(&buffer) {
                state.geomag_data = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::PRESSURE != 0 {
            // Procesar datos de presión
            let mut buffer = [0u8; dmp_packet_bytes::PRESSURE];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::PRESSURE]);
            idx += dmp_packet_bytes::PRESSURE;
            state.pressure_data = Some(buffer[0..6].try_into().unwrap());

        }
        if state.last_header & dmp_header::GYRO_CALIBR != 0 {
            // Procesar datos de giroscopio BIAS
            let mut buffer = [0u8; dmp_packet_bytes::GYRO_CALIBR];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::GYRO_CALIBR]);
            idx += dmp_packet_bytes::GYRO_CALIBR;
            if let Some(calibr) = process_quaternion_6(&buffer) {
                state.gyro_calibr = Some(calibr);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::COMPASS_CALIBR != 0 {
            // Procesar datos de calibración del compás
            let mut buffer = [0u8; dmp_packet_bytes::COMPASS_CALIBR];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::COMPASS_CALIBR]);
            idx += dmp_packet_bytes::COMPASS_CALIBR;
            if let Some(calibr) = process_quaternion_6(&buffer) {
                state.compass_calibr = Some(calibr);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if state.last_header & dmp_header::STEP_DETECTOR != 0 {
            // Procesar datos del detector de pasos
            let mut buffer = [0u8; dmp_packet_bytes::STEP_DETECTOR];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::STEP_DETECTOR]);
            idx += dmp_packet_bytes::STEP_DETECTOR;
            state.pedometer_timestamp = Some(u32::from_be_bytes(buffer[0..4].try_into().unwrap()));
        }

        if state.last_header & dmp_header::HEADER2 != 0 {
            if state.last_header2 & dmp_header2::ACCEL_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::ACCEL_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::ACCEL_ACCURACY]);
                idx += dmp_packet_bytes2::ACCEL_ACCURACY;
                state.accel_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if state.last_header2 & dmp_header2::GYRO_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::GYRO_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::GYRO_ACCURACY]);
                idx += dmp_packet_bytes2::GYRO_ACCURACY;
                state.gyro_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if state.last_header2 & dmp_header2::COMPASS_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::COMPASS_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::COMPASS_ACCURACY]);
                idx += dmp_packet_bytes2::COMPASS_ACCURACY;
                state.compass_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if state.last_header2 & dmp_header2::FSYNC != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::FSYNC];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::FSYNC]);
                idx += dmp_packet_bytes2::FSYNC;
                state.fsync = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if state.last_header2 & dmp_header2::PICKUP != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::PICKUP];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::PICKUP]);
                idx += dmp_packet_bytes2::PICKUP;
                state.pickup_state = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()) != 0);
            }
            if state.last_header2 & dmp_header2::ACTIVITY_RECOG != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::ACTIVITY_RECOG];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::ACTIVITY_RECOG]);
                idx += dmp_packet_bytes2::ACTIVITY_RECOG;
                let activity_data = ActRecognitionData {
                    state_start: buffer[0],
                    state_end: buffer[1],
                    timestamp: u32::from_be_bytes(buffer[2..6].try_into().unwrap()),
                };
                state.activity_recognition = Some(activity_data);
            }
            if state.last_header2 & dmp_header2::SECONDARY_ON_OFF != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::SECONDARY_ON_OFF];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::SECONDARY_ON_OFF]);
                idx += dmp_packet_bytes2::SECONDARY_ON_OFF;
                state.secondary_on_off = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            // Limpiar el header2
            state.last_header2 = dmp_header::EMPTY;
        }
        // Por último leer el footer del FIFO
        if idx + 2 > packet.len() {
            // Si no hay suficientes datos para leer el footer, comprobar si el fifo dispone de datos
            let retry_count = self.device.get_fifo_count()?;
            if retry_count < 2 {
                return Err(DmpFifoError::NoDataAvailable);
            }
            *count = 2; //Leer el footer y liberamos el FIFO
        }
        let mut footer = [0u8; 2];
        footer.copy_from_slice(&packet[idx..idx + 2]);
        state.footer = u16::from_be_bytes(footer);

        // Actualizar timestamp y contador de paquetes
        state.last_timestamp_us = timestamp_now;
        state.packets_processed += 1;
        state.count = *count;

        // Reiniciar el header para la próxima lectura
        state.last_header = dmp_header::EMPTY;
        
        Ok(())
    }
    
    /// Obtiene la orientación (ángulos de Euler) a partir de los datos del DMP
    pub fn get_dmp_orientation(&self, state: &DmpFifoState) -> Result<[f32; 3], DmpFifoError> {
        // Intentar usar cuaternión de 9 ejes si está disponible
        if let Some(quat) = &state.quaternion9 {
            return Ok(quaternion_to_euler(quat));
        }
        
        // De lo contrario, usar cuaternión de 6 ejes
        if let Some(quat) = &state.quaternion6 {
            return Ok(quaternion_to_euler(quat));
        }
        
        Err(DmpFifoError::NoDataAvailable)
    }
    
    /// Obtiene el cuaternión de 6 ejes (Game Rotation Vector)
    pub fn get_dmp_quaternion6(&self, state: &DmpFifoState) -> Result<Quaternion, DmpFifoError> {
        if let Some(quat) = &state.quaternion6 {
            Ok(*quat)
        } else {
            Err(DmpFifoError::NoDataAvailable)
        }
    }
    
    /// Obtiene el cuaternión de 9 ejes (Rotation Vector)
    pub fn get_dmp_quaternion9(&self, state: &DmpFifoState) -> Result<Quaternion, DmpFifoError> {
        if let Some(quat) = &state.quaternion9 {
            Ok(*quat)
        } else {
            Err(DmpFifoError::NoDataAvailable)
        }
    }
    
    /// Configura la interrupción DMP
    pub fn set_int1_assertion(&mut self, enable: i32) -> Result<(), Icm20948Error> {
        // Leer registro INT_ENABLE
        let mut int_enable = self.device.read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)?;
        
        if enable != 0 {
            // Habilitar interrupción DMP
            int_enable |= bits::DMP_INT_EN;
        } else {
            // Deshabilitar interrupción DMP
            int_enable &= !bits::DMP_INT_EN;
        }
        
        // Escribir configuración actualizada
        self.device.write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE, int_enable)?;
        
        Ok(())
    }
}
