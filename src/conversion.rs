//! Funciones de conversión para datos del sensor ICM20948
//!
//! Este módulo proporciona funciones para convertir datos raw del sensor
//! a unidades físicas apropiadas como aceleraciones en G, velocidad angular
//! en grados por segundo, y temperatura en grados Celsius.

use crate::types::AccelFullScale;
use crate::types::GyroFullScale;

/// Convierte un valor de punto fijo a flotante con el desplazamiento especificado
/// 
/// # Arguments
/// * `value` - El valor en punto fijo
/// * `shift` - El número de bits para el desplazamiento (Q format)
///
/// # Returns
/// El valor convertido a flotante
#[inline]
pub fn fixed_to_float(value: i32, shift: u8) -> f32 {
    (value as f32) / (1u32 << shift) as f32
}

/// Convierte un valor de brújula desde unidades DMP a flotante
#[inline]
pub fn dmp_to_float_compass(value: i32) -> f32 {
    fixed_to_float(value, 16)
}

/// Convierte datos brutos de acelerómetro a G según la escala configurada
/// 
/// # Arguments
/// * `raw` - Valores brutos del acelerómetro [x, y, z]
/// * `scale` - Configuración de escala completa del acelerómetro
///
/// # Returns
/// Aceleración en G [x, y, z]
pub fn accel_raw_to_g(raw: [i16; 3], scale: AccelFullScale) -> [f32; 3] {
    let factor = match scale {
        AccelFullScale::Fs2G => 2.0 / 32768.0,
        AccelFullScale::Fs4G => 4.0 / 32768.0,
        AccelFullScale::Fs8G => 8.0 / 32768.0,
        AccelFullScale::Fs16G => 16.0 / 32768.0,
    };
    
    [
        raw[0] as f32 * factor,
        raw[1] as f32 * factor,
        raw[2] as f32 * factor,
    ]
}

/// Convierte datos brutos de giroscopio a grados/segundo según la escala configurada
/// 
/// # Arguments
/// * `raw` - Valores brutos del giroscopio [x, y, z]
/// * `scale` - Configuración de escala completa del giroscopio
///
/// # Returns
/// Velocidad angular en grados/segundo [x, y, z]
pub fn gyro_raw_to_dps(raw: [i16; 3], scale: GyroFullScale) -> [f32; 3] {
    let factor = match scale {
        GyroFullScale::Fs250Dps => 250.0 / 32768.0,
        GyroFullScale::Fs500Dps => 500.0 / 32768.0,
        GyroFullScale::Fs1000Dps => 1000.0 / 32768.0,
        GyroFullScale::Fs2000Dps => 2000.0 / 32768.0,
    };
    
    [
        raw[0] as f32 * factor,
        raw[1] as f32 * factor,
        raw[2] as f32 * factor,
    ]
}

/// Convierte datos brutos de temperatura a grados Celsius
/// 
/// # Arguments
/// * `raw` - Valor bruto del sensor de temperatura
///
/// # Returns
/// Temperatura en grados Celsius
pub fn temp_raw_to_celsius(raw: i16) -> f32 {
    // La fórmula se toma del datasheet del ICM20948:
    // Temp °C = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21°C
    // Para el ICM20948: RoomTemp_Offset=0, Temp_Sensitivity=333.87
    (raw as f32 - 21.0) / 333.87 + 21.0
}

/// Convierte datos de cuaternión DMP a valores normalizados de cuaternión
/// 
/// # Arguments
/// * `raw` - Valores brutos del cuaternión [w, x, y, z]
///
/// # Returns
/// Cuaternión normalizado [w, x, y, z]
pub fn dmp_quat_to_float(raw: [i32; 4]) -> [f32; 4] {
    // En DMP, los valores de cuaternión están en formato Q30
    let q30 = 1 << 30;
    [
        (raw[0] as f32) / (q30 as f32),
        (raw[1] as f32) / (q30 as f32),
        (raw[2] as f32) / (q30 as f32), 
        (raw[3] as f32) / (q30 as f32),
    ]
}

/// Convierte ángulos de Euler DMP a grados
/// 
/// # Arguments
/// * `raw` - Ángulos de Euler en formato DMP [roll, pitch, yaw]
///
/// # Returns
/// Ángulos en grados [roll, pitch, yaw]
pub fn dmp_euler_to_degrees(raw: [i32; 3]) -> [f32; 3] {
    // En DMP, los ángulos de Euler están en formato Q16
    let q16 = 1 << 16;
    [
        (raw[0] as f32) * 180.0 / (q16 as f32) / std::f32::consts::PI,
        (raw[1] as f32) * 180.0 / (q16 as f32) / std::f32::consts::PI,
        (raw[2] as f32) * 180.0 / (q16 as f32) / std::f32::consts::PI,
    ]
}

/// Convierte gravedad DMP a m/s²
/// 
/// # Arguments
/// * `raw` - Valores de gravedad en formato DMP [x, y, z]
///
/// # Returns
/// Valores de gravedad en m/s² [x, y, z]
pub fn dmp_gravity_to_ms2(raw: [i32; 3]) -> [f32; 3] {
    // La gravedad en DMP está en formato Q16
    let q16 = 1 << 16;
    let g = 9.81; // Aceleración debida a la gravedad en m/s²
    
    [
        (raw[0] as f32) * g / (q16 as f32),
        (raw[1] as f32) * g / (q16 as f32),
        (raw[2] as f32) * g / (q16 as f32),
    ]
}

/// Convierte aceleración lineal DMP a m/s²
/// 
/// # Arguments
/// * `raw` - Valores de aceleración lineal en formato DMP [x, y, z]
///
/// # Returns
/// Valores de aceleración lineal en m/s² [x, y, z]
pub fn dmp_linear_accel_to_ms2(raw: [i32; 3]) -> [f32; 3] {
    // La aceleración lineal en DMP está en formato Q16
    let q16 = 1 << 16;
    let g = 9.81; // Aceleración debida a la gravedad en m/s²
    
    [
        (raw[0] as f32) * g / (q16 as f32),
        (raw[1] as f32) * g / (q16 as f32),
        (raw[2] as f32) * g / (q16 as f32),
    ]
}
