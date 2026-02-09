//! Implementación para manejar el FIFO del DMP en ICM20948
use crate::base::{SystemTimeSource, TimeSource};
use crate::device::Icm20948Error;
use crate::dmp::DmpDriverIcm20948;
use crate::interface::Interface;
use crate::register::{dmp, registers::bank0};
use crate::types::{bits, dmp_header, dmp_header2, dmp_packet_bytes, dmp_packet_bytes2};
use core::fmt::Debug;
use embedded_hal::delay::DelayNs;

pub const PI: f32 = 3.14159265358979323846;
// extern crate alloc;
// use alloc::vec::Vec;
use log;

use crate::conversion;
#[cfg(feature = "rhai")]
use std::sync::{OnceLock, RwLock};

#[inline]
fn fifo_debug_enabled() -> bool {
    matches!(std::env::var("MPU_FIFO_DEBUG"), Ok(val) if val == "1" || val == "true")
}

#[inline]
fn fifo_dump_enabled() -> bool {
    matches!(std::env::var("MPU_FIFO_DUMP"), Ok(val) if val == "1" || val == "true")
}

fn fifo_dump_max() -> usize {
    std::env::var("MPU_FIFO_DUMP_MAX")
        .ok()
        .and_then(|val| val.parse::<usize>().ok())
        .filter(|val| *val > 0)
        .unwrap_or(256)
}

fn dump_fifo_bytes(buf: &[u8]) -> String {
    let max = fifo_dump_max();
    let dump_len = buf.len().min(max);
    let mut out = String::new();
    for (line_idx, chunk) in buf[..dump_len].chunks(16).enumerate() {
        if line_idx > 0 {
            out.push('\n');
        }
        for (i, b) in chunk.iter().enumerate() {
            if i > 0 {
                out.push(' ');
            }
            out.push_str(&format!("{:02X}", b));
        }
    }
    out
}

#[inline]
fn header_mask() -> u16 {
    dmp_header::PED_STEPIND
        | dmp_header::ACCEL
        | dmp_header::GYRO
        | dmp_header::COMPASS
        | dmp_header::ALS
        | dmp_header::QUAT6
        | dmp_header::QUAT9
        | dmp_header::PQUAT6
        | dmp_header::GEOMAG
        | dmp_header::PRESSURE
        | dmp_header::GYRO_CALIBR
        | dmp_header::COMPASS_CALIBR
        | dmp_header::STEP_DETECTOR
        | dmp_header::HEADER2
}

#[inline]
fn header2_mask() -> u16 {
    dmp_header2::ACCEL_ACCURACY
        | dmp_header2::GYRO_ACCURACY
        | dmp_header2::COMPASS_ACCURACY
        | dmp_header2::FSYNC
        | dmp_header2::PICKUP
        | dmp_header2::ACTIVITY_RECOG
        | dmp_header2::SECONDARY_ON_OFF
        | dmp_header2::SCREEN_ROTATION
        | dmp_header2::BATCH_MODE_EN
}

#[inline]
fn is_header_valid(header: u16) -> bool {
    if header == 0 || header == dmp_header::EMPTY {
        return false;
    }
    if (header & !header_mask()) != 0 {
        return false;
    }
    true
}

#[inline]
fn is_header2_valid(header2: u16) -> bool {
    header2 != 0 && header2 != dmp_header::EMPTY && (header2 & !header2_mask()) == 0
}

#[inline]
fn calc_payload_size(header: u16, header2: u16) -> usize {
    let mut size = 0usize;
    for i in 0..16 {
        size += match header & (1 << i) {
            dmp_header::ACCEL => dmp_packet_bytes::RAW_ACCEL,
            dmp_header::GYRO => dmp_packet_bytes::RAW_GYRO + dmp_packet_bytes::GYRO_BIAS,
            dmp_header::COMPASS => dmp_packet_bytes::COMPASS,
            dmp_header::ALS => dmp_packet_bytes::ALS,
            dmp_header::QUAT6 => dmp_packet_bytes::QUAT6,
            dmp_header::QUAT9 => dmp_packet_bytes::QUAT9,
            dmp_header::PQUAT6 => dmp_packet_bytes::PQUAT6,
            dmp_header::GEOMAG => dmp_packet_bytes::GEOMAG,
            dmp_header::PRESSURE => dmp_packet_bytes::PRESSURE,
            // GYRO_CALIBR se usa como flag; el FIFO no incluye payload dedicado.
            dmp_header::GYRO_CALIBR => 0,
            dmp_header::COMPASS_CALIBR => dmp_packet_bytes::COMPASS_CALIBR,
            dmp_header::STEP_DETECTOR => dmp_packet_bytes::STEP_DETECTOR,
            _ => 0,
        };
    }

    if header & dmp_header::HEADER2 != 0 {
        for i in 0..16 {
            size += match header2 & (1 << i) {
                dmp_header2::ACCEL_ACCURACY => dmp_packet_bytes2::ACCEL_ACCURACY,
                dmp_header2::GYRO_ACCURACY => dmp_packet_bytes2::GYRO_ACCURACY,
                dmp_header2::COMPASS_ACCURACY => dmp_packet_bytes2::COMPASS_ACCURACY,
                dmp_header2::FSYNC => dmp_packet_bytes2::FSYNC,
                dmp_header2::PICKUP => dmp_packet_bytes2::PICKUP,
                dmp_header2::ACTIVITY_RECOG => dmp_packet_bytes2::ACTIVITY_RECOG,
                dmp_header2::SECONDARY_ON_OFF => dmp_packet_bytes2::SECONDARY_ON_OFF,
                dmp_header2::SCREEN_ROTATION => dmp_packet_bytes2::SCREEN_ROTATION,
                dmp_header2::BATCH_MODE_EN => dmp_packet_bytes2::BATCH_MODE,
                _ => 0,
            };
        }
    }

    size += dmp_packet_bytes2::ODR_CNT_GYRO;
    size
}

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

#[cfg(feature = "rhai")]
static MOUNT_MATRIX: OnceLock<RwLock<[i8; 9]>> = OnceLock::new();

#[cfg(feature = "rhai")]
static MOUNT_QUAT_CORRECTION: OnceLock<RwLock<bool>> = OnceLock::new();

#[cfg(feature = "rhai")]
pub fn set_mount_matrix(matrix: [i8; 9]) {
    let m_lock = MOUNT_MATRIX.get_or_init(|| RwLock::new([1, 0, 0, 0, 1, 0, 0, 0, 1]));
    if let Ok(mut guard) = m_lock.write() {
        *guard = matrix;
    }
}

#[cfg(feature = "rhai")]
pub fn set_mount_quaternion_correction(enabled: bool) {
    let lock = MOUNT_QUAT_CORRECTION.get_or_init(|| RwLock::new(true));
    if let Ok(mut guard) = lock.write() {
        *guard = enabled;
    }
}

#[cfg(feature = "rhai")]
fn mount_quaternion_correction_enabled() -> bool {
    let lock = MOUNT_QUAT_CORRECTION.get_or_init(|| RwLock::new(true));
    if let Ok(guard) = lock.read() {
        *guard
    } else {
        true
    }
}

#[cfg(not(feature = "rhai"))]
fn mount_quaternion_correction_enabled() -> bool {
    true
}

#[cfg(feature = "rhai")]
fn get_mount_matrix() -> [i8; 9] {
    let lock = MOUNT_MATRIX.get_or_init(|| RwLock::new([1, 0, 0, 0, 1, 0, 0, 0, 1]));
    if let Ok(guard) = lock.read() {
        *guard
    } else {
        [1, 0, 0, 0, 1, 0, 0, 0, 1]
    }
}

#[cfg(not(feature = "rhai"))]
fn get_mount_matrix() -> [i8; 9] {
    [1, 0, 0, 0, 1, 0, 0, 0, 1]
}

#[cfg(feature = "rhai")]
fn apply_mount_flip(vec: [f32; 3]) -> [f32; 3] {
    let m = get_mount_matrix();
    [
        m[0] as f32 * vec[0] + m[1] as f32 * vec[1] + m[2] as f32 * vec[2],
        m[3] as f32 * vec[0] + m[4] as f32 * vec[1] + m[5] as f32 * vec[2],
        m[6] as f32 * vec[0] + m[7] as f32 * vec[1] + m[8] as f32 * vec[2],
    ]
}

#[cfg(feature = "rhai")]
pub fn apply_mount_flip_vec(vec: [f32; 3]) -> [f32; 3] {
    apply_mount_flip(vec)
}

pub fn is_rotation_matrix(matrix: [i8; 9]) -> bool {
    let m = [
        matrix[0] as i32,
        matrix[1] as i32,
        matrix[2] as i32,
        matrix[3] as i32,
        matrix[4] as i32,
        matrix[5] as i32,
        matrix[6] as i32,
        matrix[7] as i32,
        matrix[8] as i32,
    ];
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6]);
    det == 1
}

pub fn quaternion_from_mount_matrix(matrix: [i8; 9]) -> Option<Quaternion> {
    if !is_rotation_matrix(matrix) {
        return None;
    }

    let r00 = matrix[0] as f32;
    let r01 = matrix[1] as f32;
    let r02 = matrix[2] as f32;
    let r10 = matrix[3] as f32;
    let r11 = matrix[4] as f32;
    let r12 = matrix[5] as f32;
    let r20 = matrix[6] as f32;
    let r21 = matrix[7] as f32;
    let r22 = matrix[8] as f32;

    let trace = r00 + r11 + r22;
    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s)
    } else if r00 > r11 && r00 > r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        ((r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s)
    } else if r11 > r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        ((r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s)
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        ((r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s)
    };

    let q = normalize_quaternion(w, x, y, z);
    Some(Quaternion {
        w: q.0,
        x: q.1,
        y: q.2,
        z: q.3,
        heading_accuracy_deg: None,
    })
}

#[derive(Debug, Clone, Copy)]
pub struct ActRecognitionData {
    pub state_start: u8, // Tipo de actividad (byte[0])
    pub state_end: u8,   // Tipo de actividad anterior (byte[1])
    pub timestamp: u32,  // Timestamp (byte[2..5])
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
    /// Habilitar detección de pickup
    pub pickup_enable: bool,
    /// Habilitar detección de actividad
    pub activity_enable: bool,
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
            pickup_enable: false,
            activity_enable: false,
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
    /// Calibración del giroscopio (Q20)
    pub gyro_calibr: Option<[i32; 3]>,
    /// Calibración del compás (Q16)
    pub compass_calibr: Option<[i32; 3]>,
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
    /// ODR counter (footer)
    pub odr_counter: Option<u16>,
    /// Estado de detección de pickup
    pub pickup_state: Option<bool>,
    /// Estado de rotación de pantalla
    pub screen_rotation: Option<u8>,
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
    /// Buffer de bytes pendiente de procesar (FIFO)
    pub fifo_buf: Vec<u8>,
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
            odr_counter: None,
            pickup_state: None,
            screen_rotation: None,
            activity_recognition: None,
            secondary_on_off: None,
            footer: 0,
            packets_processed: 0,
            packet_size: 0,
            packet: Vec::new(),
            fifo_buf: Vec::new(),
            count: 0,
            last_timestamp_us: 0,
        }
    }
}

/// Extrae un valor i32 de 4 bytes en formato big endian
#[inline]
fn extract_i32_big_endian(buffer: &[u8], offset: usize) -> i32 {
    ((buffer[offset] as i32) << 24)
        | ((buffer[offset + 1] as i32) << 16)
        | ((buffer[offset + 2] as i32) << 8)
        | (buffer[offset + 3] as i32)
}

/// Extrae un valor i16 de 2 bytes en formato big endian
#[inline]
fn extract_i16_big_endian(buffer: &[u8], offset: usize) -> i16 {
    ((buffer[offset] as i16) << 8) | (buffer[offset + 1] as i16)
}

#[inline]
fn quat_from_vector_q30(q1: i32, q2: i32, q3: i32) -> (f32, f32, f32, f32) {
    let scale = 1.0_f64 / ((1u64 << 30) as f64);
    let mut x = q1 as f64 * scale;
    let mut y = q2 as f64 * scale;
    let mut z = q3 as f64 * scale;

    let sum = x * x + y * y + z * z;
    if sum > 1.0 {
        let inv = 1.0 / sum.sqrt();
        x *= inv;
        y *= inv;
        z *= inv;
    }

    let w = (1.0 - (x * x + y * y + z * z)).max(0.0).sqrt();
    (w as f32, x as f32, y as f32, z as f32)
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

    // Q1, Q2, Q3 son el vector del cuaternión (Q0 se calcula)
    let q1 = extract_i32_big_endian(fifo_data, 0);
    let q2 = extract_i32_big_endian(fifo_data, 4);
    let q3 = extract_i32_big_endian(fifo_data, 8);

    let (w, x, y, z) = quat_from_vector_q30(q1, q2, q3);

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

/// Procesa los datos de calibración (vector 3x i32 en Q format)
pub fn process_calibrated_vec_i32(fifo_data: &[u8]) -> Option<[i32; 3]> {
    if fifo_data.len() < 12 {
        return None;
    }

    let v0 = extract_i32_big_endian(fifo_data, 0);
    let v1 = extract_i32_big_endian(fifo_data, 4);
    let v2 = extract_i32_big_endian(fifo_data, 8);
    Some([v0, v1, v2])
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
    let sum_squares = x * x + y * y + z * z;
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

    // Q1, Q2, Q3 son el vector del cuaternión (Q0 se calcula)
    let q1 = extract_i32_big_endian(fifo_data, 0);
    let q2 = extract_i32_big_endian(fifo_data, 4);
    let q3 = extract_i32_big_endian(fifo_data, 8);

    // Extraer la precisión del rumbo (heading accuracy)
    let heading_accuracy_raw = extract_i16_big_endian(fifo_data, 12);

    let (w, x, y, z) = quat_from_vector_q30(q1, q2, q3);

    // Normalizar el cuaternión para corregir el drift
    let quaternion = normalize_quaternion(w, x, y, z);

    // Heading accuracy viene en Q29, pero el FIFO solo entrega los 16 bits altos.
    // Resultado: accuracy_deg = (raw << 16) / 2^29 = raw / 2^13
    let heading_accuracy_deg = (heading_accuracy_raw as f32) / 8192.0;

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
    let magnitude = (w * w + x * x + y * y + z * z).sqrt();

    if magnitude > 1e-6 {
        // Evitar división por cero
        let inv_magnitude = 1.0 / magnitude;
        (
            w * inv_magnitude,
            x * inv_magnitude,
            y * inv_magnitude,
            z * inv_magnitude,
        )
    } else {
        (1.0, 0.0, 0.0, 0.0) // Valor predeterminado en caso de magnitud casi cero
    }
}

/// Rotaciona un cuaternión a partir de un cuaternión de referencia
/// # Arguments
/// * `q` - Cuaternión a rotacionar
/// * `q_ref` - Cuaternión de referencia
///
/// # Returns
/// Cuaternión rotacionado

pub fn rotate_quaternion(q: &Quaternion, q_ref: &Quaternion) -> Quaternion {
    let w = q.w * q_ref.w - q.x * q_ref.x - q.y * q_ref.y - q.z * q_ref.z;
    let x = q.w * q_ref.x + q.x * q_ref.w + q.y * q_ref.z - q.z * q_ref.y;
    let y = q.w * q_ref.y - q.x * q_ref.z + q.y * q_ref.w + q.z * q_ref.x;
    let z = q.w * q_ref.z + q.x * q_ref.y - q.y * q_ref.x + q.z * q_ref.w;

    Quaternion {
        w,
        x,
        y,
        z,
        heading_accuracy_deg: None,
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
    quaternion_to_euler_dmp(quaternion)
}

/// Calculates Euler angles using the DMP orientation convention.
/// Returns [roll, pitch, yaw] in degrees.
pub fn quaternion_to_euler_dmp(quaternion: &Quaternion) -> [f32; 3] {
    let q = apply_mount_correction_to_quaternion(quaternion);

    // Row-major rotation matrix (same layout used by InvenSense augmented path)
    let w = q.w;
    let x = q.x;
    let y = q.y;
    let z = q.z;

    let m00 = 1.0 - 2.0 * (y * y + z * z);
    let _m01 = 2.0 * (x * y - w * z);
    let _m02 = 2.0 * (x * z + w * y);
    let m10 = 2.0 * (x * y + w * z);
    let _m11 = 1.0 - 2.0 * (x * x + z * z);
    let _m12 = 2.0 * (y * z - w * x);
    let m20 = 2.0 * (x * z - w * y);
    let m21 = 2.0 * (y * z + w * x);
    let m22 = 1.0 - 2.0 * (x * x + y * y);

    let yaw = (-m10).atan2(m00);
    let pitch = (-m21).atan2(m22);
    let roll = (m20).atan2((1.0 - m20 * m20).sqrt());

    let rad_to_deg = 180.0 / std::f32::consts::PI;
    [roll * rad_to_deg, pitch * rad_to_deg, yaw * rad_to_deg]
}

// fn quaternion_to_euler_deg(q: &Quaternion) -> (f32, f32, f32) {
//     let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
//     let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
//     let roll = sinr_cosp.atan2(cosr_cosp).to_degrees();

//     let sinp = 2.0 * (q.w * q.y - q.z * q.x);
//     let pitch = if sinp.abs() >= 1.0 {
//         sinp.signum() * 90.0 // Gimbal lock
//     } else {
//         sinp.asin().to_degrees()
//     };

//     let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
//     let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
//     let yaw = siny_cosp.atan2(cosy_cosp).to_degrees();

//     (roll, pitch, yaw)
// }

// pub fn quaternion_to_angles_lockless(q: &Quaternion) -> [f32; 3] {
//     // Calculamos la "gravedad" proyectada desde el cuaternión
//     let gx = q.w * q.y - q.x * q.z;
//     let gy = -(q.y * q.z + q.w * q.x);
//     let gz = 0.5 - q.w * q.w - q.z * q.z;

//     // Roll y pitch desde el vector de gravedad
//     let roll_rad = f32::atan2(gx, (gy * gy + gz * gz).sqrt());
//     let pitch_rad = -f32::atan2(gy, gz) - PI;

//     // Yaw desde el cuaternión (como siempre)
//     let yaw_rad = f32::atan2(q.w * q.z + q.x * q.y, 0.5 - q.y * q.y - q.z * q.z);

//     // Conversión a grados
//     let mut roll_deg = roll_rad.to_degrees();
//     let mut pitch_deg = pitch_rad.to_degrees();
//     let mut yaw_deg = -yaw_rad.to_degrees();

//     // Corrección cuando el sensor está boca abajo
//     // if gz > 0.0 {
//     //     if roll_deg > 0.0 {
//     //         roll_deg = 180.0 - roll_deg;
//     //     } else {
//     //         roll_deg = -180.0 - roll_deg;
//     //     }
//     // }

//     // Ajustes para mantener los valores entre [-180, 180]
//     // roll_deg = if roll_deg < -180.0 {
//     //     roll_deg + 360.0
//     // } else {
//     //     roll_deg
//     // };
//     // pitch_deg = if pitch_deg < -180.0 {
//     //     pitch_deg + 360.0
//     // } else {
//     //     pitch_deg
//     // };
//     // yaw_deg = if yaw_deg > 360.0 {
//     //     yaw_deg - 360.0
//     // } else {
//     //     yaw_deg
//     // };

//     [roll_deg, pitch_deg, yaw_deg]
// }

#[inline]
fn qmul(a: &Quaternion, b: &Quaternion) -> Quaternion {
    Quaternion {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        heading_accuracy_deg: None,
    }
}

#[inline]
fn qconj(q: &Quaternion) -> Quaternion {
    Quaternion {
        w: q.w,
        x: -q.x,
        y: -q.y,
        z: -q.z,
        heading_accuracy_deg: None,
    }
}

#[inline]
fn apply_mount_correction_to_quaternion(q: &Quaternion) -> Quaternion {
    if !mount_quaternion_correction_enabled() {
        return *q;
    }
    // Use the full mount matrix to compute the correction quaternion
    let matrix = get_mount_matrix();
    if let Some(q_mount) = quaternion_from_mount_matrix(matrix) {
        // Match InvenSense conversion path: q_body_to_world = q_chip_to_world * inverse(q_chip_to_body)
        qmul(q, &qconj(&q_mount))
    } else {
        *q
    }
}

pub fn quaternion_to_angles_lockless(q_dmp: &Quaternion) -> [f32; 3] {
    let q = apply_mount_correction_to_quaternion(q_dmp);

    // Roll (X), Pitch (Y), Yaw (Z) – convención típica
    let sinr = 2.0 * (q.w * q.x + q.y * q.z);
    let cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let roll = sinr.atan2(cosr);

    let sinp = 2.0 * (q.w * q.y - q.z * q.x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * core::f32::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };

    let siny = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    let yaw = siny.atan2(cosy);

    [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()]
}

/// Calcula el quaternión desde ángulos de Euler (roll, pitch, yaw)
/// /// # Arguments
/// * `roll` - Ángulo de roll en radianes
/// * `pitch` - Ángulo de pitch en radianes
/// * `yaw` - Ángulo de yaw en radianes
/// # Returns
/// Estructura Quaternion con los datos calculados
pub fn euler_to_quaternion(roll: f32, pitch: f32, yaw: f32) -> Quaternion {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();

    let w = cy * cr * cp + sy * sr * sp;
    let x = cy * sr * cp - sy * cr * sp;
    let y = cy * cr * sp + sy * sr * cp;
    let z = sy * cr * cp - cy * sr * sp;

    Quaternion {
        w,
        x,
        y,
        z,
        heading_accuracy_deg: None,
    }
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

    Some(conversion::dmp_linear_accel_to_ms2([
        accel_x, accel_y, accel_z,
    ]))
}

/// Implementación de funciones para el FIFO del DMP
impl<I, D, E> DmpDriverIcm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayNs,
    // E: Debug,
    // Icm20948Error: From<E>,
{
    fn refresh_output_masks(&mut self) {
        let prev1 = self.output_mask1;
        let prev2 = self.output_mask2;
        let mut buf = [0u8; 2];
        if self
            .device
            .read_mems(dmp::data_output_control::DATA_OUT_CTL1, &mut buf)
            .is_ok()
        {
            self.output_mask1 = u16::from_be_bytes(buf);
        }
        if self
            .device
            .read_mems(dmp::data_output_control::DATA_OUT_CTL2, &mut buf)
            .is_ok()
        {
            self.output_mask2 = u16::from_be_bytes(buf);
        }
        if fifo_debug_enabled() && (self.output_mask1 != prev1 || self.output_mask2 != prev2) {
            log::debug!(
                "MPU FIFO output masks: ctl1=0x{:04X} ctl2=0x{:04X}",
                self.output_mask1,
                self.output_mask2
            );
        }
    }
    /// Reset el FIFO del DMP
    pub fn reset_fifo(&mut self) -> Result<(), Icm20948Error> {
        // Resetear el FIFO
        self.device
            .reset_fifo()
            .map_err(|_| Icm20948Error::InvalidOperation)?;

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
        self.device
            .enable_fifo(enable)
            .map_err(|_| Icm20948Error::InvalidOperation)?;
        self.device
            .reset_fifo()
            .map_err(|_| Icm20948Error::InvalidOperation)?;

        Ok(())
    }

    /// Lee el registro de estado de interrupción (INT_STATUS)
    pub fn read_int_status_flags(&mut self) -> Result<u8, Icm20948Error> {
        self.device.read_mems_reg::<bank0::Bank>(bank0::INT_STATUS)
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
            state.fifo_buf.clear();
            return Err(DmpFifoError::FifoOverflow);
        }

        if fifo_count == 0 && state.fifo_buf.is_empty() {
            return Err(DmpFifoError::NoDataAvailable);
        }

        // Procesar datos
        self.process_dmp_fifo_data(state, &mut fifo_count)?;

        Ok(())
    }

    /// Procesa los datos del FIFO del DMP
    fn process_dmp_fifo_data(
        &mut self,
        state: &mut DmpFifoState,
        count: &mut usize,
    ) -> Result<(), DmpFifoError> {
        let timestamp_now = SystemTimeSource.get_timestamp_us();

        self.refresh_output_masks();

        if *count > 0 {
            let mut buf = vec![0u8; *count];
            self.device
                .read_mems_regs::<bank0::Bank>(bank0::FIFO_R_W, &mut buf)?;
            state.fifo_buf.extend_from_slice(&buf);
            *count = 0;
        }

        let mut parsed_any = false;
        loop {
            if state.fifo_buf.len() < 2 {
                break;
            }

            let header = u16::from_be_bytes([state.fifo_buf[0], state.fifo_buf[1]]);
            let allowed_header = if self.output_mask1 != 0 {
                self.output_mask1 | dmp_header::PED_STEPIND
            } else {
                header_mask()
            };
            if !is_header_valid(header) || (header & !allowed_header) != 0 {
                if fifo_debug_enabled() {
                    let dump_len = state.fifo_buf.len().min(8);
                    log::warn!(
                        "MPU FIFO desync: invalid header 0x{:04X}, buf[0..{}]={:02X?}",
                        header,
                        dump_len,
                        &state.fifo_buf[0..dump_len]
                    );
                    if fifo_dump_enabled() {
                        let dump = dump_fifo_bytes(&state.fifo_buf);
                        log::warn!(
                            "MPU FIFO raw dump len={} (showing up to {} bytes):\n{}",
                            state.fifo_buf.len(),
                            fifo_dump_max(),
                            dump
                        );
                    }
                }
                self.device.reset_fifo().ok();
                state.fifo_buf.clear();
                return Err(DmpFifoError::InvalidPacket);
            }

            let has_header2 = (header & dmp_header::HEADER2) != 0;
            if has_header2 && state.fifo_buf.len() < 4 {
                break;
            }

            let mut header2 = if has_header2 {
                u16::from_be_bytes([state.fifo_buf[2], state.fifo_buf[3]])
            } else {
                dmp_header::EMPTY
            };

            let allowed_header2 = if self.output_mask2 != 0 {
                self.output_mask2
            } else {
                header2_mask()
            };
            if has_header2
                && (!is_header2_valid(header2)
                    || (header2 & !header2_mask()) != 0
                    || (header2 & !allowed_header2) != 0)
            {
                let swapped = header2.swap_bytes();
                if is_header2_valid(swapped)
                    && (swapped & !header2_mask()) == 0
                    && (swapped & !allowed_header2) == 0
                {
                    if fifo_debug_enabled() {
                        log::warn!(
                            "MPU FIFO header2 byte order swap detected: 0x{:04X} -> 0x{:04X}",
                            header2,
                            swapped
                        );
                    }
                    header2 = swapped;
                } else {
                    if fifo_debug_enabled() {
                        log::warn!(
                            "MPU FIFO desync: invalid header2 0x{:04X} for header 0x{:04X}",
                            header2,
                            header
                        );
                        if fifo_dump_enabled() {
                            let dump = dump_fifo_bytes(&state.fifo_buf);
                            log::warn!(
                                "MPU FIFO raw dump len={} (showing up to {} bytes):\n{}",
                                state.fifo_buf.len(),
                                fifo_dump_max(),
                                dump
                            );
                        }
                    }
                    self.device.reset_fifo().ok();
                    state.fifo_buf.clear();
                    return Err(DmpFifoError::InvalidPacket);
                }
            }

            let payload_size = calc_payload_size(header, header2);
            let total_size = 2 + if has_header2 { 2 } else { 0 } + payload_size;
            if state.fifo_buf.len() < total_size {
                if fifo_debug_enabled() {
                    log::debug!(
                        "MPU FIFO awaiting packet: header=0x{:04X} header2=0x{:04X} need={} have={}",
                        header,
                        header2,
                        total_size,
                        state.fifo_buf.len()
                    );
                }
                break;
            }

            let payload_start = 2 + if has_header2 { 2 } else { 0 };
            let payload_end = payload_start + payload_size;
            let payload = state.fifo_buf[payload_start..payload_end].to_vec();

            state.header = header;
            state.header2 = header2;
            state.packet_size = total_size;
            state.packet.clear();
            state.packet.extend(state.header.to_be_bytes());
            if has_header2 {
                state.packet.extend(state.header2.to_be_bytes());
            }
            state.packet.extend_from_slice(&payload);

            if fifo_debug_enabled() {
                log::debug!(
                    "MPU FIFO packet: header=0x{:04X} header2=0x{:04X} size={}",
                    header,
                    header2,
                    total_size
                );
            }

            self.decode_packet_from_bytes(state, header, header2, &payload)?;
            parsed_any = true;
            state.packets_processed += 1;

            state.fifo_buf.drain(0..total_size);
        }

        if !parsed_any {
            return Err(DmpFifoError::NoDataAvailable);
        }

        state.last_timestamp_us = timestamp_now;
        state.count = *count;

        Ok(())
    }

    fn decode_packet_from_bytes(
        &self,
        state: &mut DmpFifoState,
        header: u16,
        header2: u16,
        packet: &[u8],
    ) -> Result<(), DmpFifoError> {
        let mut idx = 0;

        if header & dmp_header::ACCEL != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::RAW_ACCEL];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::RAW_ACCEL]);
            idx += dmp_packet_bytes::RAW_ACCEL;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            state.accel_data = Some([x, y, z]);
        }
        if header & dmp_header::GYRO != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::RAW_GYRO];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::RAW_GYRO]);
            idx += dmp_packet_bytes::RAW_GYRO;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            state.gyro_data = Some([x, y, z]);

            let mut bias_buf = [0u8; dmp_packet_bytes::GYRO_BIAS];
            bias_buf.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::GYRO_BIAS]);
            idx += dmp_packet_bytes::GYRO_BIAS;
            let bx = i16::from_be_bytes(bias_buf[0..2].try_into().unwrap());
            let by = i16::from_be_bytes(bias_buf[2..4].try_into().unwrap());
            let bz = i16::from_be_bytes(bias_buf[4..6].try_into().unwrap());
            state.gyro_bias = Some([bx, by, bz]);
        }
        if header & dmp_header::COMPASS != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::COMPASS];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::COMPASS]);
            idx += dmp_packet_bytes::COMPASS;
            let x = i16::from_be_bytes(buffer[0..2].try_into().unwrap());
            let y = i16::from_be_bytes(buffer[2..4].try_into().unwrap());
            let z = i16::from_be_bytes(buffer[4..6].try_into().unwrap());
            state.compass_data = Some([x, y, z]);
        }
        if header & dmp_header::ALS != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::ALS];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::ALS]);
            idx += dmp_packet_bytes::ALS;
        }
        if header & dmp_header::QUAT6 != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::QUAT6];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::QUAT6]);
            idx += dmp_packet_bytes::QUAT6;
            if let Some(quat) = process_quaternion_6(&buffer) {
                state.quaternion6 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if header & dmp_header::QUAT9 != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::QUAT9];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::QUAT9]);
            idx += dmp_packet_bytes::QUAT9;
            if let Some(quat) = process_quaternion_9(&buffer) {
                state.quaternion9 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if header & dmp_header::PQUAT6 != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::PQUAT6];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::PQUAT6]);
            idx += dmp_packet_bytes::PQUAT6;
            if let Some(quat) = process_quaternion_p6(&buffer) {
                state.quaternionp6 = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if header & dmp_header::GEOMAG != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::GEOMAG];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::GEOMAG]);
            idx += dmp_packet_bytes::GEOMAG;
            if let Some(quat) = process_quaternion_9(&buffer) {
                state.geomag_data = Some(quat);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if header & dmp_header::PRESSURE != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::PRESSURE];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::PRESSURE]);
            idx += dmp_packet_bytes::PRESSURE;
            state.pressure_data = Some(buffer[0..6].try_into().unwrap());
        }
        if header & dmp_header::GYRO_CALIBR != 0 {
            // No payload dedicado; se deriva con gyro - bias si están presentes.
            if let (Some(gyro), Some(bias)) = (state.gyro_data, state.gyro_bias) {
                let gx = (gyro[0] as i32 - bias[0] as i32) * 32;
                let gy = (gyro[1] as i32 - bias[1] as i32) * 32;
                let gz = (gyro[2] as i32 - bias[2] as i32) * 32;
                state.gyro_calibr = Some([gx, gy, gz]);
            }
        }
        if header & dmp_header::COMPASS_CALIBR != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::COMPASS_CALIBR];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::COMPASS_CALIBR]);
            idx += dmp_packet_bytes::COMPASS_CALIBR;
            if let Some(calibr) = process_calibrated_vec_i32(&buffer) {
                state.compass_calibr = Some(calibr);
            } else {
                return Err(DmpFifoError::DataError);
            }
        }
        if header & dmp_header::STEP_DETECTOR != 0 {
            let mut buffer = [0u8; dmp_packet_bytes::STEP_DETECTOR];
            buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes::STEP_DETECTOR]);
            idx += dmp_packet_bytes::STEP_DETECTOR;
            state.pedometer_timestamp = Some(u32::from_be_bytes(buffer[0..4].try_into().unwrap()));
        }

        if header & dmp_header::HEADER2 != 0 {
            if header2 & dmp_header2::ACCEL_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::ACCEL_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::ACCEL_ACCURACY]);
                idx += dmp_packet_bytes2::ACCEL_ACCURACY;
                state.accel_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if header2 & dmp_header2::GYRO_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::GYRO_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::GYRO_ACCURACY]);
                idx += dmp_packet_bytes2::GYRO_ACCURACY;
                state.gyro_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if header2 & dmp_header2::COMPASS_ACCURACY != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::COMPASS_ACCURACY];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::COMPASS_ACCURACY]);
                idx += dmp_packet_bytes2::COMPASS_ACCURACY;
                state.compass_accuracy = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if header2 & dmp_header2::FSYNC != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::FSYNC];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::FSYNC]);
                idx += dmp_packet_bytes2::FSYNC;
                state.fsync = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if header2 & dmp_header2::PICKUP != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::PICKUP];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::PICKUP]);
                idx += dmp_packet_bytes2::PICKUP;
                state.pickup_state =
                    Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()) != 0);
            }
            if header2 & dmp_header2::ACTIVITY_RECOG != 0 {
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
            if header2 & dmp_header2::SECONDARY_ON_OFF != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::SECONDARY_ON_OFF];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::SECONDARY_ON_OFF]);
                idx += dmp_packet_bytes2::SECONDARY_ON_OFF;
                state.secondary_on_off = Some(u16::from_be_bytes(buffer[0..2].try_into().unwrap()));
            }
            if header2 & dmp_header2::BATCH_MODE_EN != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::BATCH_MODE];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::BATCH_MODE]);
                idx += dmp_packet_bytes2::BATCH_MODE;
                // Valor de batch mode no expuesto por ahora.
                let _ = u16::from_be_bytes(buffer[0..2].try_into().unwrap());
            }
            if header2 & dmp_header2::SCREEN_ROTATION != 0 {
                let mut buffer = [0u8; dmp_packet_bytes2::SCREEN_ROTATION];
                buffer.copy_from_slice(&packet[idx..idx + dmp_packet_bytes2::SCREEN_ROTATION]);
                idx += dmp_packet_bytes2::SCREEN_ROTATION;
                let val = u16::from_be_bytes(buffer[0..2].try_into().unwrap());
                #[cfg(debug_assertions)]
                println!("MPU FIFO: Screen Rotation Event: {}", val);
                state.screen_rotation = Some(val as u8);
            }

            let known_bits = dmp_header2::SECONDARY_ON_OFF
                | dmp_header2::ACTIVITY_RECOG
                | dmp_header2::BATCH_MODE_EN
                | dmp_header2::PICKUP
                | dmp_header2::FSYNC
                | dmp_header2::COMPASS_ACCURACY
                | dmp_header2::GYRO_ACCURACY
                | dmp_header2::ACCEL_ACCURACY
                | dmp_header2::SCREEN_ROTATION;

            let unknown_bits = header2 & !known_bits;
            if unknown_bits != 0 {
                #[cfg(debug_assertions)]
                println!(
                    "MPU FIFO: Unknown Header2 bits: 0x{:04X}. Idx: {}",
                    unknown_bits, idx
                );
            }
        }

        if idx + dmp_packet_bytes2::ODR_CNT_GYRO > packet.len() {
            return Err(DmpFifoError::InsufficientData);
        }
        let odr = u16::from_be_bytes(
            packet[idx..idx + dmp_packet_bytes2::ODR_CNT_GYRO]
                .try_into()
                .unwrap(),
        );
        state.odr_counter = Some(odr);
        state.footer = odr;

        Ok(())
    }

    /// Obtiene la orientación (ángulos de Euler) a partir de los datos del DMP
    pub fn get_dmp_orientation(&self, state: &DmpFifoState) -> Result<[f32; 3], DmpFifoError> {
        // Intentar usar cuaternión de 9 ejes si está disponible
        if let Some(quat) = &state.quaternion9 {
            return Ok(quaternion_to_euler_dmp(quat));
        }

        // De lo contrario, usar cuaternión de 6 ejes
        if let Some(quat) = &state.quaternion6 {
            return Ok(quaternion_to_euler_dmp(quat));
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
        let mut int_enable = self
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)?;

        if enable != 0 {
            // Habilitar interrupción DMP
            int_enable |= bits::DMP_INT_EN;
        } else {
            // Deshabilitar interrupción DMP
            int_enable &= !bits::DMP_INT_EN;
        }

        // Escribir configuración actualizada
        self.device
            .write_mems_reg::<bank0::Bank>(bank0::INT_ENABLE, int_enable)?;

        Ok(())
    }

    /// Lee los registros de estado de interrupción para limpiar el latch.
    pub fn clear_int_status(&mut self) -> Result<(), Icm20948Error> {
        let _ = self
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_STATUS)?;
        let _ = self
            .device
            .read_mems_reg::<bank0::Bank>(bank0::DMP_INT_STATUS)?;
        Ok(())
    }
}

impl DmpFifoState {
    pub fn clear_data(&mut self) {
        self.accel_data = None;
        self.gyro_data = None;
        self.gyro_bias = None;
        self.compass_data = None;
        self.quaternion6 = None;
        self.quaternion9 = None;
        self.quaternionp6 = None;
        self.geomag_data = None;
        self.pressure_data = None;
        self.gyro_calibr = None;
        self.compass_calibr = None;
        self.pedometer_timestamp = None;
        self.accel_accuracy = None;
        self.gyro_accuracy = None;
        self.compass_accuracy = None;
        self.fsync = None;
        self.odr_counter = None;
        self.pickup_state = None;
        self.activity_recognition = None;
        self.secondary_on_off = None;
    }
}
