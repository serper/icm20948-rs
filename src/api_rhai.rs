#[cfg(feature = "rhai")]
use crate::dmp::DmpDriverIcm20948;
#[cfg(feature = "rhai")]
use crate::dmp_fifo::{DmpFifoError, DmpFifoState};
#[cfg(feature = "rhai")]
use crate::interface::I2cInterface;
#[cfg(feature = "rhai")]
use crate::selftest;
#[cfg(feature = "rhai")]
use crate::types::{OdrSensor, Sensor};
#[cfg(feature = "rhai")]
use linux_embedded_hal::{Delay, I2cdev};
#[cfg(feature = "rhai")]
use rhai::plugin::*;
#[cfg(feature = "rhai")]
use rhai::{CustomType, EvalAltResult, TypeBuilder};
#[cfg(feature = "rhai")]
use std::fs::File;
#[cfg(feature = "rhai")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "rhai")]
type LinuxDmp = DmpDriverIcm20948<I2cInterface<I2cdev>, Delay>;

/// Wrapper to expose ICM20948 functionality to Rhai
#[cfg(feature = "rhai")]
#[derive(Clone)]
pub struct RhaiIcm20948 {
    // Shared access to the device
    inner: Arc<Mutex<LinuxDmp>>,
}

#[cfg(feature = "rhai")]
impl RhaiIcm20948 {
    pub fn new(inner: Arc<Mutex<LinuxDmp>>) -> Self {
        Self { inner }
    }
}

#[cfg(feature = "rhai")]
impl CustomType for RhaiIcm20948 {
    fn build(mut builder: TypeBuilder<Self>) {
        builder.with_name("Icm20948");
    }
}

#[cfg(feature = "rhai")]
#[export_module]
pub mod icm20948_api {
    /// Initializes the ICM20948 sensor.
    ///
    /// # Syntax
    /// icm.initialize()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:1
    #[rhai_fn(global, return_raw)]
    pub fn initialize(icm: &mut RhaiIcm20948) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.initialize().map_err(|e| format!("{:?}", e).into())
    }

    /// Enables or disables the DMP (Digital Motion Processor).
    ///
    /// # Syntax
    /// icm.enable_dmp(enable)
    /// # Parameters:
    /// - icm: Icm20948
    /// - enable: bool
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:2
    #[rhai_fn(global, return_raw)]
    pub fn enable_dmp(icm: &mut RhaiIcm20948, enable: bool) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.enable_dmp(enable)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Enables or disables a specific sensor.
    ///
    /// # Syntax
    /// icm.enable_sensor(sensor_name, enable)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor_name: string ("accel", "gyro", "compass", "quat6", "quat9", "geomag")
    /// - enable: bool
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:3
    #[rhai_fn(global, return_raw)]
    pub fn enable_sensor(
        icm: &mut RhaiIcm20948,
        sensor_name: &str,
        enable: bool,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let sensor = match sensor_name {
            "accel" => Sensor::Accelerometer,
            "gyro" => Sensor::Gyroscope,
            "compass" => Sensor::GeomagneticField,
            "quat6" => Sensor::GameRotationVector,
            "quat9" => Sensor::RotationVector,
            "geomag" => Sensor::GeomagneticRotationVector,
            _ => {
                return Err(format!(
                    "Unknown or unsupported sensor for enabling: {}",
                    sensor_name
                )
                .into())
            }
        };
        dmp.enable_dmp_sensor(sensor, enable)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Sets the data rate for a specific sensor.
    ///
    /// # Syntax
    /// icm.set_sensor_rate(sensor_name, rate)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor_name: string ("accel", "gyro", "compass", "quat6", "quat9", "geomag", "als", "pressure", "gyro_calibr", "compass_calibr")
    /// - rate: int (divider value)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:4
    #[rhai_fn(global, return_raw)]
    pub fn set_sensor_rate(
        icm: &mut RhaiIcm20948,
        sensor_name: &str,
        rate: i64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let sensor = match sensor_name {
            "accel" => OdrSensor::Accel,
            "gyro" => OdrSensor::Gyro,
            "compass" => OdrSensor::Cpass,
            "quat6" => OdrSensor::Quat6,
            "quat9" => OdrSensor::Quat9,
            "geomag" => OdrSensor::Geomag,
            "als" => OdrSensor::Als,
            "pressure" => OdrSensor::Pressure,
            "gyro_calibr" => OdrSensor::GyroCalibr,
            "compass_calibr" => OdrSensor::CpassCalibr,
            _ => return Err(format!("Unknown sensor for rate: {}", sensor_name).into()),
        };
        dmp.set_sensor_rate(sensor, rate as i16)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Calibrates the gyroscope.
    ///
    /// # Syntax
    /// icm.calibrate_gyro()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:5
    #[rhai_fn(global, return_raw)]
    pub fn calibrate_gyro(icm: &mut RhaiIcm20948) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        selftest::calibrate(&mut dmp.device, true).map_err(|e| format!("{:?}", e).into())
    }

    /// Calibrates the accelerometer.
    ///
    /// # Syntax
    /// icm.calibrate_accel()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:6
    #[rhai_fn(global, return_raw)]
    pub fn calibrate_accel(icm: &mut RhaiIcm20948) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        selftest::calibrate(&mut dmp.device, true).map_err(|e| format!("{:?}", e).into())
    }

    /// Reads data from the DMP FIFO.
    ///
    /// # Syntax
    /// icm.read_data()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map (containing "accel", "gyro", "quat" keys with data)
    /// # rhai-autodocs:index:7
    #[rhai_fn(global, return_raw)]
    pub fn read_data(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let mut state = DmpFifoState::default();

        match dmp.read_dmp_fifo_data(&mut state) {
            Ok(_) => {}
            Err(DmpFifoError::NoDataAvailable) => return Ok(rhai::Map::new()),
            Err(e) => return Err(format!("Error reading FIFO: {:?}", e).into()),
        }

        let mut map = rhai::Map::new();

        if let Some(accel) = state.accel_data {
            let mut accel_map = rhai::Map::new();
            accel_map.insert("x".into(), (accel[0] as f64).into());
            accel_map.insert("y".into(), (accel[1] as f64).into());
            accel_map.insert("z".into(), (accel[2] as f64).into());
            map.insert("accel".into(), accel_map.into());
        }

        if let Some(gyro) = state.gyro_data {
            let mut gyro_map = rhai::Map::new();
            gyro_map.insert("x".into(), (gyro[0] as f64).into());
            gyro_map.insert("y".into(), (gyro[1] as f64).into());
            gyro_map.insert("z".into(), (gyro[2] as f64).into());
            map.insert("gyro".into(), gyro_map.into());
        }

        if let Some(quat) = state.quaternion9.or(state.quaternion6) {
            let mut quat_map = rhai::Map::new();
            quat_map.insert("w".into(), (quat.w as f64).into());
            quat_map.insert("x".into(), (quat.x as f64).into());
            quat_map.insert("y".into(), (quat.y as f64).into());
            quat_map.insert("z".into(), (quat.z as f64).into());
            map.insert("quat".into(), quat_map.into());
        }

        Ok(map)
    }

    /// Saves calibration data to a file.
    ///
    /// # Syntax
    /// icm.save_calibration(filepath)
    /// # Parameters:
    /// - icm: Icm20948
    /// - filepath: string
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:8
    #[rhai_fn(global, return_raw)]
    pub fn save_calibration(
        icm: &mut RhaiIcm20948,
        filepath: &str,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let accel_bias = dmp.get_bias_acc().map_err(|e| format!("{:?}", e))?;
        let gyro_bias = dmp.get_bias_gyr().map_err(|e| format!("{:?}", e))?;
        let compass_bias = dmp.get_bias_cmp().map_err(|e| format!("{:?}", e))?;

        let data = serde_json::json!({
            "accel_bias": accel_bias,
            "gyro_bias": gyro_bias,
            "compass_bias": compass_bias
        });

        let file = File::create(filepath).map_err(|e| e.to_string())?;
        serde_json::to_writer(file, &data).map_err(|e| e.to_string())?;

        Ok(())
    }

    /// Loads calibration data from a file.
    ///
    /// # Syntax
    /// icm.load_calibration(filepath)
    /// # Parameters:
    /// - icm: Icm20948
    /// - filepath: string
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:9
    #[rhai_fn(global, return_raw)]
    pub fn load_calibration(
        icm: &mut RhaiIcm20948,
        filepath: &str,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let file = File::open(filepath).map_err(|e| e.to_string())?;
        let data: serde_json::Value = serde_json::from_reader(file).map_err(|e| e.to_string())?;

        if let Some(accel_bias) = data["accel_bias"].as_array() {
            let bias = [
                accel_bias[0].as_i64().unwrap_or(0) as i32,
                accel_bias[1].as_i64().unwrap_or(0) as i32,
                accel_bias[2].as_i64().unwrap_or(0) as i32,
            ];
            dmp.set_bias_acc(&bias).map_err(|e| format!("{:?}", e))?;
        }

        if let Some(gyro_bias) = data["gyro_bias"].as_array() {
            let bias = [
                gyro_bias[0].as_i64().unwrap_or(0) as i32,
                gyro_bias[1].as_i64().unwrap_or(0) as i32,
                gyro_bias[2].as_i64().unwrap_or(0) as i32,
            ];
            dmp.set_bias_gyr(&bias).map_err(|e| format!("{:?}", e))?;
        }

        if let Some(compass_bias) = data["compass_bias"].as_array() {
            let bias = [
                compass_bias[0].as_i64().unwrap_or(0) as i32,
                compass_bias[1].as_i64().unwrap_or(0) as i32,
                compass_bias[2].as_i64().unwrap_or(0) as i32,
            ];
            dmp.set_bias_cmp(&bias).map_err(|e| format!("{:?}", e))?;
        }

        Ok(())
    }
}

/// Registers the ICM20948 API into the Rhai engine
#[cfg(feature = "rhai")]
pub fn register_icm_api(engine: &mut rhai::Engine) {
    engine.build_type::<RhaiIcm20948>();
    engine.register_global_module(exported_module!(icm20948_api).into());
}
