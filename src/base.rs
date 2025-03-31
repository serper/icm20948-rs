//! Funcionalidades y traits base para módulos del sensor

/// Trait para obtener un timestamp en microsegundos.
/// Permite implementar diferentes fuentes (sistema o sensor).
pub trait TimeSource {
    /// Retorna el timestamp (en microsegundos)
    fn get_timestamp_us(&self) -> u64;
}

/// Implementación por defecto usando el reloj del sistema.
pub struct SystemTimeSource;

impl TimeSource for SystemTimeSource {
    fn get_timestamp_us(&self) -> u64 {
        // Utilizamos std::time para obtener el tiempo en microsegundos.
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Tiempo invertido")
            .as_micros() as u64
    }
}

/// Implementación para cuando se disponga de un timestamp proporcionado por el sensor.
pub struct SensorTimeSource {
    pub sensor_timestamp: u64,
}

impl TimeSource for SensorTimeSource {
    fn get_timestamp_us(&self) -> u64 {
        // Retorna el timestamp entregado por el sensor.
        self.sensor_timestamp
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_system_time_source() {
        let ts = SystemTimeSource;
        let t = ts.get_timestamp_us();
        println!("Timestamp sistema: {}", t);
        assert!(t > 0);
    }
    
    #[test]
    fn test_sensor_time_source() {
        let ts = SensorTimeSource { sensor_timestamp: 123456789 };
        assert_eq!(ts.get_timestamp_us(), 123456789);
    }
}
