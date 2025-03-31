//! Utilidades comunes para los ejemplos

use std::time::{Duration, Instant};
use icm20948_rs::{base::TimeSource, device::Icm20948Error};

/// Implementación de TimeSource para los ejemplos
pub struct StdTimeSource {
    start: Instant,
}

impl StdTimeSource {
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
    
    /// Obtener la marca de tiempo base
    pub fn get_base_timestamp(&self) -> Instant {
        self.start
    }
    
    /// Reiniciar la marca de tiempo
    pub fn reset_timestamp(&mut self) {
        self.start = Instant::now();
    }
}

impl<'a> TimeSource<'a> for StdTimeSource {
    fn get_timestamp_us(&'a self) -> u64 {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64
    }
}

/// Helper para manejar errores en los ejemplos
pub fn handle_error<T>(result: Result<T, Icm20948Error>) -> T {
    match result {
        Ok(val) => val,
        Err(e) => {
            eprintln!("Error: {:?}", e);
            std::process::exit(1);
        }
    }
}

/// Función para pausar la ejecución por un tiempo determinado
pub fn delay_ms(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}
