//! Implementación de sensores aumentados para ICM20948
//!
//! Este módulo proporciona funcionalidades para obtener datos de "sensores virtuales"
//! como gravedad, aceleración lineal y orientación, que son calculados a partir de
//! los datos de los sensores físicos y procesamiento adicional.

use crate::device::Icm20948;
use crate::dmp_fifo::{QuaternionData, OrientationData};
use crate::controls::{AccelData, GyroData};
use core::f32::consts::PI;
use crate::types::{AccelFullScale, GyroFullScale};

/// Factor para convertir radianes a grados
const RAD_TO_DEG: f32 = 180.0 / PI;
/// Factor para convertir grados a radianes
const DEG_TO_RAD: f32 = PI / 180.0;

/// Datos de gravedad extraídos del cuaternión de orientación
pub struct GravityData {
    /// Vector de gravedad (g) [x, y, z]
    pub gravity: [f32; 3],
    /// Marca de tiempo en microsegundos
    pub timestamp_us: u64,
}

/// Datos de aceleración lineal (sin componente gravitacional)
pub struct LinearAccelData {
    /// Vector de aceleración lineal (m/s²) [x, y, z]
    pub accel: [f32; 3],
    /// Marca de tiempo en microsegundos
    pub timestamp_us: u64,
}

/// Estado de los sensores aumentados
pub struct AugmentedState {
    /// Datos de gravedad actual
    pub gravity: GravityData,
    /// Datos de aceleración lineal actual
    pub linear_accel: LinearAccelData,
    /// Datos de orientación actual
    pub orientation: OrientationData,
    /// Último cuaternión utilizado para cálculos
    pub last_quaternion: QuaternionData,
}

impl AugmentedState {
    /// Crea una nueva instancia del estado de sensores aumentados
    pub fn new() -> Self {
        Self {
            gravity: GravityData {
                gravity: [0.0, 0.0, 1.0],
                timestamp_us: 0,
            },
            linear_accel: LinearAccelData {
                accel: [0.0, 0.0, 0.0],
                timestamp_us: 0,
            },
            orientation: OrientationData::default(),
            last_quaternion: QuaternionData::default(),
        }
    }

    /// Procesa datos del acelerómetro y devuelve datos de aceleración lineal
    pub fn process_accel(&mut self, accel_data: &AccelData) -> LinearAccelData {
        LinearAccelData {
            accel: [
                accel_data.x - self.gravity.gravity[0],
                accel_data.y - self.gravity.gravity[1],
                accel_data.z - self.gravity.gravity[2],
            ],
            timestamp_us: accel_data.timestamp_us.max(self.gravity.timestamp_us),
        }
    }

    /// Procesa datos del giroscopio
    pub fn process_gyro(&mut self, _gyro_data: &GyroData) {
        // Implementar procesamiento de datos del giroscopio
    }

    /// Obtiene el vector de gravedad actual
    pub fn get_gravity(&self, _timestamp_us: u64) -> GravityData {
        GravityData {
            gravity: self.gravity.gravity,
            timestamp_us: _timestamp_us,
        }
    }

    /// Convierte datos del acelerómetro a m/s² considerando la escala
    pub fn convert_accel_to_ms2(&self, accel_data: &AccelData, scale: AccelFullScale) -> [f32; 3] {
        let scale_factor = match scale {
            AccelFullScale::Fs2G => 16384.0,
            AccelFullScale::Fs4G => 8192.0,
            AccelFullScale::Fs8G => 4096.0,
            AccelFullScale::Fs16G => 2048.0,
        };
        
        let g_to_ms2 = 9.81; // Conversion de g a m/s²
        
        [
            accel_data.x / scale_factor * g_to_ms2,
            accel_data.y / scale_factor * g_to_ms2,
            accel_data.z / scale_factor * g_to_ms2,
        ]
    }
    
    /// Convierte datos del giroscopio a rad/s considerando la escala
    pub fn convert_gyro_to_rads(&self, gyro_data: &GyroData, scale: GyroFullScale) -> [f32; 3] {
        let scale_factor = match scale {
            GyroFullScale::Fs250Dps => 131.0,
            GyroFullScale::Fs500Dps => 65.5,
            GyroFullScale::Fs1000Dps => 32.8,
            GyroFullScale::Fs2000Dps => 16.4,
        };
        
        let dps_to_rads = PI / 180.0; // Conversion de grados/s a radianes/s
        
        [
            gyro_data.x / scale_factor * dps_to_rads,
            gyro_data.y / scale_factor * dps_to_rads,
            gyro_data.z / scale_factor * dps_to_rads,
        ]
    }
}

impl Default for AugmentedState {
    fn default() -> Self {
        Self::new()
    }
}

/// Implementación de sensores aumentados para ICM20948
impl<I2C, D, E> Icm20948<I2C, D>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E> + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    /// Obtiene la gravedad a partir de datos de cuaternión de 6 ejes
    pub fn get_gravity_from_quat6(&mut self, quat: &QuaternionData) -> GravityData {
        // Para cuaternión de 6-ejes, solo tenemos componentes x, y, z (no w)
        // Reconstruimos w para tener un cuaternión unitario
        let qw = compute_quat_w(quat.x, quat.y, quat.z);
        let qw_sq = qw * qw;
        
        // Calcular componentes de gravedad (rotación del vector [0, 0, 1] por el cuaternión)
        let mut gravity = GravityData {
            gravity: [0.0, 0.0, 1.0],
            timestamp_us: quat.timestamp_us,
        };
        
        gravity.gravity[0] = 2.0 * (quat.x * quat.z - qw * quat.y);
        gravity.gravity[1] = 2.0 * (qw * quat.x + quat.y * quat.z);
        gravity.gravity[2] = qw_sq - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z;
        
        gravity
    }
    
    /// Obtiene la gravedad a partir de datos de cuaternión de 9 ejes
    pub fn get_gravity_from_quat9(&mut self, quat: &QuaternionData) -> GravityData {
        // Para cuaternión de 9-ejes, tenemos los 4 componentes del cuaternión
        let qw = quat.w;
        let qw_sq = qw * qw;
        
        // Calcular componentes de gravedad
        let mut gravity = GravityData {
            gravity: [0.0, 0.0, 1.0],
            timestamp_us: quat.timestamp_us,
        };
        
        gravity.gravity[0] = 2.0 * (quat.x * quat.z - qw * quat.y);
        gravity.gravity[1] = 2.0 * (qw * quat.x + quat.y * quat.z);
        gravity.gravity[2] = qw_sq - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z;
        
        gravity
    }
    
    /// Calcula la aceleración lineal a partir de aceleración total y gravedad
    pub fn get_linear_acceleration(
        &mut self,
        accel: &AccelData,
        gravity: &GravityData
    ) -> LinearAccelData {
        // Aceleración lineal = Aceleración total - Gravedad
        LinearAccelData {
            accel: [
                accel.x - gravity.gravity[0],
                accel.y - gravity.gravity[1],
                accel.z - gravity.gravity[2],
            ],
            timestamp_us: accel.timestamp_us.max(gravity.timestamp_us),
        }
    }
    
    /// Obtiene orientación en ángulos de Euler (roll, pitch, yaw) a partir de cuaternión de 9 ejes
    pub fn get_orientation_from_quat9(&mut self, quat: &QuaternionData) -> OrientationData {
        // Convertir cuaternión a ángulos de Euler
        let (roll, pitch, mut yaw) = quaternion_to_euler(quat.w, quat.x, quat.y, quat.z);
        
        // Normalizar yaw a [0, 360)
        if yaw < 0.0 {
            yaw += 360.0;
        }
        
        OrientationData {
            roll,
            pitch,
            yaw,
            heading_accuracy: 0.0,  // En una implementación real, esto dependería de la precisión del magnetómetro
            timestamp_us: quat.timestamp_us,
        }
    }
    
    /// Obtiene orientación en ángulos de Euler a partir de cuaternión de 6 ejes
    pub fn get_orientation_from_quat6(&mut self, quat: &QuaternionData) -> OrientationData {
        // Reconstruir qw para cuaternión de 6 ejes
        let qw = compute_quat_w(quat.x, quat.y, quat.z);
        
        // Convertir cuaternión a ángulos de Euler
        let (roll, pitch, mut yaw) = quaternion_to_euler(qw, quat.x, quat.y, quat.z);
        
        // Normalizar yaw a [0, 360)
        if yaw < 0.0 {
            yaw += 360.0;
        }
        
        OrientationData {
            roll,
            pitch,
            yaw,
            heading_accuracy: 0.0,
            timestamp_us: quat.timestamp_us,
        }
    }
    
    /// Procesa los datos de los sensores para actualizar sensores aumentados
    pub fn update_augmented_sensors(
        &mut self,
        state: &mut AugmentedState,
        quat: &QuaternionData,
        accel: &AccelData
    ) {
        // Guardar cuaternión
        state.last_quaternion = *quat;
        
        // Calcular gravedad (según tipo de cuaternión)
        if quat.w > 0.0 || quat.w < 0.0 {
            // Cuaternión de 9-ejes
            state.gravity = self.get_gravity_from_quat9(quat);
            state.orientation = self.get_orientation_from_quat9(quat);
        } else {
            // Cuaternión de 6-ejes
            state.gravity = self.get_gravity_from_quat6(quat);
            state.orientation = self.get_orientation_from_quat6(quat);
        }
        
        // Calcular aceleración lineal
        state.linear_accel = self.get_linear_acceleration(accel, &state.gravity);
    }
    
    /// Calcula la matriz de rotación a partir de cuaternión
    pub fn quaternion_to_rotation_matrix(&mut self, quat: &QuaternionData) -> [[f32; 3]; 3] {
        let mut qw = quat.w;
        let qx = quat.x;
        let qy = quat.y;
        let qz = quat.z;
        
        // Si es cuaternión de 6 ejes (w = 0), reconstruir w
        if qw == 0.0 {
            qw = compute_quat_w(qx, qy, qz);
        }
        
        // Crear matriz de rotación a partir del cuaternión
        let mut matrix = [[0.0; 3]; 3];
        
        let qw2 = qw * qw;
        let qx2 = qx * qx;
        let qy2 = qy * qy;
        let qz2 = qz * qz;
        
        // Diagonal principal
        matrix[0][0] = qw2 + qx2 - qy2 - qz2;
        matrix[1][1] = qw2 - qx2 + qy2 - qz2;
        matrix[2][2] = qw2 - qx2 - qy2 + qz2;
        
        // Elementos fuera de la diagonal
        let qxy = qx * qy;
        let qwz = qw * qz;
        let qxz = qx * qz;
        let qwy = qw * qy;
        let qyz = qy * qz;
        let qwx = qw * qx;
        
        matrix[0][1] = 2.0 * (qxy - qwz);
        matrix[1][0] = 2.0 * (qxy + qwz);
        matrix[0][2] = 2.0 * (qxz + qwy);
        matrix[2][0] = 2.0 * (qxz - qwy);
        matrix[1][2] = 2.0 * (qyz - qwx);
        matrix[2][1] = 2.0 * (qyz + qwx);
        
        matrix
    }
}

/// Reconstruye la componente W de un cuaternión a partir de X, Y, Z
/// asumiendo que es un cuaternión unitario (|q| = 1)
fn compute_quat_w(x: f32, y: f32, z: f32) -> f32 {
    let w_squared = 1.0 - x * x - y * y - z * z;
    
    if w_squared < 0.0 {
        0.0  // Error numérico, debería ser un cuaternión unitario
    } else {
        w_squared.sqrt()
    }
}

/// Convierte un cuaternión a ángulos de Euler (roll, pitch, yaw) en grados
fn quaternion_to_euler(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    // Roll (rotación en X)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp) * RAD_TO_DEG;
    
    // Pitch (rotación en Y)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        // Caso especial: en los polos (±90°)
        sinp.signum() * 90.0
    } else {
        sinp.asin() * RAD_TO_DEG
    };
    
    // Yaw (rotación en Z)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp) * RAD_TO_DEG;
    
    (roll, pitch, yaw)
}

/// Funciones adicionales para cálculos con cuaterniones
pub mod quat {
    use super::*;
    
    /// Multiplica dos cuaterniones q1 * q2
    pub fn multiply(q1: &QuaternionData, q2: &QuaternionData) -> QuaternionData {
        let mut q1w = q1.w;
        let q1x = q1.x;
        let q1y = q1.y;
        let q1z = q1.z;
        
        // Si es cuaternión de 6 ejes (w = 0), reconstruir w
        if q1w == 0.0 {
            q1w = compute_quat_w(q1x, q1y, q1z);
        }
        
        let mut q2w = q2.w;
        let q2x = q2.x;
        let q2y = q2.y;
        let q2z = q2.z;
        
        // Si es cuaternión de 6 ejes (w = 0), reconstruir w
        if q2w == 0.0 {
            q2w = compute_quat_w(q2x, q2y, q2z);
        }
        
        // Multiplicación de cuaterniones
        QuaternionData {
            w: q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
            x: q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
            y: q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
            z: q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w,
            accuracy_rad: q1.accuracy_rad.max(q2.accuracy_rad),
            timestamp_us: q1.timestamp_us.max(q2.timestamp_us),
        }
    }
    
    /// Calcula el conjugado de un cuaternión
    pub fn conjugate(q: &QuaternionData) -> QuaternionData {
        QuaternionData {
            w: q.w,
            x: -q.x,
            y: -q.y,
            z: -q.z,
            accuracy_rad: q.accuracy_rad,
            timestamp_us: q.timestamp_us,
        }
    }
    
    /// Normaliza un cuaternión para que sea unitario (|q| = 1)
    pub fn normalize(q: &QuaternionData) -> QuaternionData {
        let mut qw = q.w;
        let qx = q.x;
        let qy = q.y;
        let qz = q.z;
        
        // Si es cuaternión de 6 ejes (w = 0), reconstruir w
        if qw == 0.0 {
            qw = compute_quat_w(qx, qy, qz);
        }
        
        // Calcular magnitud
        let magnitude = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
        
        if magnitude < 1e-6 {
            // Evitar división por cero
            QuaternionData::default()
        } else {
            // Normalizar
            QuaternionData {
                w: qw / magnitude,
                x: qx / magnitude,
                y: qy / magnitude,
                z: qz / magnitude,
                accuracy_rad: q.accuracy_rad,
                timestamp_us: q.timestamp_us,
            }
        }
    }
    
    /// Rota un vector por un cuaternión
    pub fn rotate_vector(q: &QuaternionData, v: [f32; 3]) -> [f32; 3] {
        // Construir cuaternión del vector (0, v)
        let mut qv = QuaternionData {
            w: 0.0,
            x: v[0],
            y: v[1],
            z: v[2],
            accuracy_rad: 0.0,
            timestamp_us: 0,
        };
        
        // Rotación: q * v * q^-1
        let q_conj = conjugate(q);
        qv = multiply(q, &qv);
        qv = multiply(&qv, &q_conj);
        
        [qv.x, qv.y, qv.z]
    }
    
    /// Convierte ángulos de Euler a cuaternión
    pub fn from_euler(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> QuaternionData {
        // Convertir a radianes
        let roll = roll_deg * DEG_TO_RAD;
        let pitch = pitch_deg * DEG_TO_RAD;
        let yaw = yaw_deg * DEG_TO_RAD;
        
        // Calcular senos y cosenos
        let (sr, cr) = (roll * 0.5).sin_cos();
        let (sp, cp) = (pitch * 0.5).sin_cos();
        let (sy, cy) = (yaw * 0.5).sin_cos();
        
        // Calcular cuaternión
        QuaternionData {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
            accuracy_rad: 0.0,
            timestamp_us: 0,
        }
    }
}
