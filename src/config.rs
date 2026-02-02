use crate::device::Icm20948Error;
use crate::dmp::DmpDriverIcm20948;
use crate::register::dmp;
use crate::types;
use crate::interface::Interface;
use embedded_hal::delay::DelayNs;

/// Enumeración para los rangos de escala completa del acelerómetro
#[derive(Debug, Clone, Copy)]
pub enum AccelFullScale {
    /// ±2g
    Fs2G,
    /// ±4g
    Fs4G,
    /// ±8g
    Fs8G,
    /// ±16g
    Fs16G,
}

/// Enumeración para los rangos de escala completa del giroscopio
#[derive(Debug, Clone, Copy)]
pub enum GyroFullScale {
    /// ±250dps
    Fs250Dps,
    /// ±500dps
    Fs500Dps,
    /// ±1000dps
    Fs1000Dps,
    /// ±2000dps
    Fs2000Dps,
}

/// Implementaciones para el controlador DMP
impl<I, D, E> DmpDriverIcm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayNs,
    Icm20948Error: From<E>,
{
    /// Inicializa las configuraciones del DMP
    pub fn initialize_dmp(&mut self) -> Result<(), Icm20948Error> {
        // Resetear registros de control
        self.reset_control_registers()?;
        
        // Configurar salidas básicas
        let output_mask_val = dmp::output_mask::ACCEL | 
                            dmp::output_mask::GYRO | 
                            dmp::output_mask::QUAT6;
        self.set_data_output_control1(output_mask_val)?;
        
        // Configurar eventos de movimiento (activar pedómetro, calibración de sensores)
        let motion_mask = dmp::motion_event_control::PEDOMETER_EN | 
                        dmp::motion_event_control::ACCEL_CAL_EN | 
                        dmp::motion_event_control::GYRO_CAL_EN;
        self.set_motion_event_control(motion_mask)?;
        
        // Configurar escalas para DMP
        self.set_gyro_fsr(2000)?; // 2000 dps
        self.set_accel_fsr(4)?;   // 4g
        
        // Configurar tasas de muestreo
        self.set_sensor_rate(types::OdrSensor::Accel, 10)?;  // 50Hz
        self.set_sensor_rate(types::OdrSensor::Gyro, 10)?;      // 50Hz
        self.set_sensor_rate(types::OdrSensor::Quat9, 10)?;          // 50Hz
        
        // Configurar umbral de Batch/FIFO
        self.set_fifo_watermark(1024)?; // Establecer cuando el FIFO está lleno a 1024 bytes
        
        Ok(())
    }

    /// Inicializa y arranca el DMP
    pub fn start_dmp(&mut self, firmware: &[u8]) -> Result<(), Icm20948Error> {
        // Cargar firmware
        self.load_firmware(firmware)?;
        
        // Inicializar configuración DMP
        self.initialize_dmp()?;
        
        // Habilitar sensor DMP y FIFO
        // Esto tendría que implementarse a nivel de dispositivo
        
        Ok(())
    }

    /// Configura una frecuencia de muestreo para el algoritmo BAC
    pub fn configure_bac(&mut self, freq: dmp::AlgoFreq) -> Result<(), Icm20948Error> {
        // Configurar la tasa BAC (Business Activity Classification)
        self.set_bac_rate(freq)?;
        
        // Restablecer los estados BAC
        self.reset_bac_states()?;
        
        // Habilitar el evento BAC en la configuración de eventos de movimiento
        let mut motion_mask = dmp::motion_event_control::PEDOMETER_EN | 
                            dmp::motion_event_control::ACCEL_CAL_EN | 
                            dmp::motion_event_control::GYRO_CAL_EN;
                            
        // Añadir la clasificación de actividad
        motion_mask |= dmp::motion_event_control::BAC_ACCEL_ONLY_EN;
        
        self.set_motion_event_control(motion_mask)?;
        
        Ok(())
    }
}
