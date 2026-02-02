#[cfg(feature = "rhai")]
use crate::dmp::DmpDriverIcm20948;
#[cfg(feature = "rhai")]
use crate::dmp_fifo::{DmpFifoError, DmpFifoState, Quaternion};
#[cfg(feature = "rhai")]
use crate::interface::I2cInterface;
#[cfg(feature = "rhai")]
use crate::register::dmp;
#[cfg(feature = "rhai")]
use crate::register::registers::bank0;
#[cfg(feature = "rhai")]
use crate::selftest;
#[cfg(feature = "rhai")]
use crate::types::{scale_factor, AccelFullScale, Activity, GyroFullScale, OdrSensor, Sensor};
#[cfg(feature = "rhai")]
use crate::{compass::CompassType, conversion};
#[cfg(feature = "rhai")]
use linux_embedded_hal::{Delay, I2cdev};
#[cfg(feature = "rhai")]
use rhai::plugin::*;
#[cfg(feature = "rhai")]
use rhai::{CustomType, EvalAltResult, TypeBuilder};
#[cfg(feature = "rhai")]
use std::fs::File;
#[cfg(feature = "rhai")]
use std::collections::HashMap;
#[cfg(feature = "rhai")]
use std::sync::{Arc, Mutex};
#[cfg(feature = "rhai")]
use once_cell::sync::Lazy;

#[cfg(feature = "rhai")]
const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;

#[cfg(feature = "rhai")]
pub type LinuxDmp = DmpDriverIcm20948<I2cInterface<I2cdev>, Delay>;

#[cfg(feature = "rhai")]
static LAST_MPU_DATA: Lazy<Mutex<HashMap<usize, rhai::Map>>> =
    Lazy::new(|| Mutex::new(HashMap::new()));

/// Wrapper to expose ICM20948 functionality to Rhai
#[cfg(feature = "rhai")]
#[derive(Clone)]
pub struct RhaiIcm20948 {
    // Shared access to the device
    pub inner: Arc<Mutex<LinuxDmp>>,
}

#[cfg(feature = "rhai")]
impl RhaiIcm20948 {
    pub fn new(inner: Arc<Mutex<LinuxDmp>>) -> Self {
        Self { inner }
    }
}

#[cfg(feature = "rhai")]
pub fn icm_cache_key(icm: &RhaiIcm20948) -> usize {
    Arc::as_ptr(&icm.inner) as usize
}

#[cfg(feature = "rhai")]
pub fn cache_last_data_for_key(key: usize, map: &rhai::Map) {
    let _ = merge_last_data_for_key(key, map);
}

#[cfg(feature = "rhai")]
pub fn merge_last_data_for_key(key: usize, map: &rhai::Map) -> rhai::Map {
    if let Ok(mut guard) = LAST_MPU_DATA.lock() {
        let entry = guard.entry(key).or_insert_with(rhai::Map::new);
        for (k, v) in map {
            entry.insert(k.clone(), v.clone());
        }
        return entry.clone();
    }
    map.clone()
}

#[cfg(feature = "rhai")]
fn get_last_data(icm: &RhaiIcm20948) -> rhai::Map {
    let key = icm_cache_key(icm);
    if let Ok(guard) = LAST_MPU_DATA.lock() {
        if let Some(map) = guard.get(&key) {
            return map.clone();
        }
    }
    rhai::Map::new()
}

#[cfg(feature = "rhai")]
fn read_mems_u16(dmp: &mut LinuxDmp, addr: u16) -> Result<u16, Box<EvalAltResult>> {
    let mut buf = [0u8; 2];
    dmp.device
        .read_mems(addr, &mut buf)
        .map_err(|e| format!("DMP mem read failed: {:?}", e))?;
    Ok(u16::from_be_bytes(buf))
}

#[cfg(feature = "rhai")]
fn vec3f_to_map(values: [f32; 3]) -> rhai::Map {
    let mut map = rhai::Map::new();
    map.insert("x".into(), (values[0] as f64).into());
    map.insert("y".into(), (values[1] as f64).into());
    map.insert("z".into(), (values[2] as f64).into());
    map
}

#[cfg(feature = "rhai")]
fn quat_to_map(quat: Quaternion) -> rhai::Map {
    let mut map = rhai::Map::new();
    map.insert("w".into(), (quat.w as f64).into());
    map.insert("x".into(), (quat.x as f64).into());
    map.insert("y".into(), (quat.y as f64).into());
    map.insert("z".into(), (quat.z as f64).into());
    if let Some(heading) = quat.heading_accuracy_deg {
        map.insert("heading_accuracy".into(), (heading as f64).into());
    }
    map
}

#[cfg(feature = "rhai")]
fn u8_array_to_rhai(values: [u8; 6]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as i64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn convert_compass_raw_to_ut(dmp: &LinuxDmp, raw: [i16; 3]) -> [f32; 3] {
    let config = dmp.device.base_state.compass_config.as_ref();
    let compass_type = config
        .map(|cfg| cfg.compass_type)
        .unwrap_or(CompassType::AK09916);

    let scale = match compass_type {
        CompassType::AK09916 => scale_factor::AK09916,
        CompassType::AK09911 => scale_factor::AK09911,
        CompassType::AK09912 => scale_factor::AK09912,
        CompassType::AK08963 => {
            if config
                .map(|cfg| cfg.calibration.scale == 0)
                .unwrap_or(true)
            {
                scale_factor::AK8963_14BIT
            } else {
                scale_factor::AK8963_16BIT
            }
        }
        _ => scale_factor::AK09916,
    } as f32
        / (1u64 << 30) as f32;

    if let Some(cfg) = config {
        let mut out = [0f32; 3];
        for i in 0..3 {
            let mut tmp_val = 0f32;
            for j in 0..3 {
                let sens_adj = (cfg.calibration.sensitivity[j] as i32 + 128) as f32;
                let matrix_val = cfg.calibration.mounting_matrix[i * 3 + j] as f32;
                tmp_val += raw[j] as f32 * matrix_val * sens_adj / 256.0;
            }
            out[i] = tmp_val * scale;
        }
        out
    } else {
        [raw[0] as f32 * scale, raw[1] as f32 * scale, raw[2] as f32 * scale]
    }
}

#[cfg(feature = "rhai")]
fn convert_accel_raw_to_g(dmp: &LinuxDmp, raw: [i16; 3]) -> [f32; 3] {
    conversion::accel_raw_to_g(raw, dmp.device.base_state.accel_fullscale)
}

#[cfg(feature = "rhai")]
fn convert_gyro_raw_to_rads(dmp: &LinuxDmp, raw: [i16; 3]) -> [f32; 3] {
    let dps = conversion::gyro_raw_to_dps(raw, dmp.device.base_state.gyro_fullscale);
    [dps[0] * DEG_TO_RAD, dps[1] * DEG_TO_RAD, dps[2] * DEG_TO_RAD]
}

#[cfg(feature = "rhai")]
pub fn fifo_state_to_map(dmp: &LinuxDmp, state: &DmpFifoState) -> rhai::Map {
    let mut map = rhai::Map::new();

    if let Some(accel) = state.accel_data {
        map.insert("accel".into(), vec3f_to_map(convert_accel_raw_to_g(dmp, accel)).into());
    }

    if let Some(gyro) = state.gyro_data {
        map.insert("gyro".into(), vec3f_to_map(convert_gyro_raw_to_rads(dmp, gyro)).into());
    }

    if let Some(gyro_bias) = state.gyro_bias {
        map.insert(
            "gyro_bias".into(),
            vec3f_to_map(convert_gyro_raw_to_rads(dmp, gyro_bias)).into(),
        );
    }

    if let Some(compass) = state.compass_data {
        map.insert(
            "compass".into(),
            vec3f_to_map(convert_compass_raw_to_ut(dmp, compass)).into(),
        );
    }

    if let Some(quat6) = state.quaternion6 {
        map.insert("quat6".into(), quat_to_map(quat6).into());
    }

    if let Some(quat9) = state.quaternion9 {
        map.insert("quat9".into(), quat_to_map(quat9).into());
    }

    if let Some(quatp6) = state.quaternionp6 {
        map.insert("quatp6".into(), quat_to_map(quatp6).into());
    }

    if let Some(geomag) = state.geomag_data {
        map.insert("geomag".into(), quat_to_map(geomag).into());
    }

    if let Some(gyro_calibr) = state.gyro_calibr {
        map.insert("gyro_calibr".into(), quat_to_map(gyro_calibr).into());
    }

    if let Some(compass_calibr) = state.compass_calibr {
        map.insert("compass_calibr".into(), quat_to_map(compass_calibr).into());
    }

    if let Some(pressure) = state.pressure_data {
        map.insert("pressure".into(), u8_array_to_rhai(pressure).into());
    }

    if let Some(timestamp) = state.pedometer_timestamp {
        map.insert("pedometer_timestamp".into(), (timestamp as i64).into());
    }

    if let Some(accel_accuracy) = state.accel_accuracy {
        map.insert("accel_accuracy".into(), (accel_accuracy as i64).into());
    }

    if let Some(gyro_accuracy) = state.gyro_accuracy {
        map.insert("gyro_accuracy".into(), (gyro_accuracy as i64).into());
    }

    if let Some(compass_accuracy) = state.compass_accuracy {
        map.insert("compass_accuracy".into(), (compass_accuracy as i64).into());
    }

    if let Some(fsync) = state.fsync {
        map.insert("fsync".into(), (fsync as i64).into());
    }

    if let Some(pickup) = state.pickup_state {
        map.insert("pickup".into(), pickup.into());
    }

    if let Some(activity) = state.activity_recognition {
        let mut act_map = rhai::Map::new();
        act_map.insert("start".into(), (activity.state_start as i64).into());
        act_map.insert("end".into(), (activity.state_end as i64).into());
        act_map.insert("time".into(), (activity.timestamp as i64).into());
        map.insert("activity".into(), act_map.into());
    }

    if let Some(secondary) = state.secondary_on_off {
        map.insert("secondary_on_off".into(), (secondary as i64).into());
    }

    if map.contains_key("quat9") {
        if let Some(value) = map.get("quat9").cloned() {
            map.insert("quat".into(), value);
        }
    } else if map.contains_key("quat6") {
        if let Some(value) = map.get("quat6").cloned() {
            map.insert("quat".into(), value);
        }
    } else if map.contains_key("quatp6") {
        if let Some(value) = map.get("quatp6").cloned() {
            map.insert("quat".into(), value);
        }
    }

    map
}

#[cfg(feature = "rhai")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuSensor(pub Sensor);

#[cfg(feature = "rhai")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuOdrSensor(pub OdrSensor);

#[cfg(feature = "rhai")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuActivity(pub Activity);

#[cfg(feature = "rhai")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuAccelScale(pub AccelFullScale);

#[cfg(feature = "rhai")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuGyroScale(pub GyroFullScale);

#[cfg(feature = "rhai")]
impl CustomType for RhaiIcm20948 {
    fn build(mut builder: TypeBuilder<Self>) {
        builder.with_name("Icm20948");
    }
}

#[cfg(feature = "rhai")]
impl CustomType for MpuSensor {
    fn build(mut builder: TypeBuilder<Self>) {
        builder
            .with_name("MpuSensor")
            .with_fn("==", |a: &mut MpuSensor, b: MpuSensor| -> bool {
                a.0 == b.0
            });
    }
}

#[cfg(feature = "rhai")]
impl CustomType for MpuOdrSensor {
    fn build(mut builder: TypeBuilder<Self>) {
        builder
            .with_name("MpuOdrSensor")
            .with_fn("==", |a: &mut MpuOdrSensor, b: MpuOdrSensor| -> bool {
                a.0 == b.0
            });
    }
}

#[cfg(feature = "rhai")]
impl CustomType for MpuActivity {
    fn build(mut builder: TypeBuilder<Self>) {
        builder
            .with_name("MpuActivity")
            .with_fn("==", |a: &mut MpuActivity, b: MpuActivity| -> bool {
                a.0 == b.0
            });
    }
}

#[cfg(feature = "rhai")]
impl CustomType for MpuAccelScale {
    fn build(mut builder: TypeBuilder<Self>) {
        builder
            .with_name("MpuAccelScale")
            .with_fn("==", |a: &mut MpuAccelScale, b: MpuAccelScale| -> bool {
                a.0 == b.0
            });
    }
}

#[cfg(feature = "rhai")]
impl CustomType for MpuGyroScale {
    fn build(mut builder: TypeBuilder<Self>) {
        builder
            .with_name("MpuGyroScale")
            .with_fn("==", |a: &mut MpuGyroScale, b: MpuGyroScale| -> bool {
                a.0 == b.0
            });
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

        // Inicializar dispositivo base primero (reset, wake up, check ID)
        dmp.device.initialize().map_err(|e| -> Box<EvalAltResult> {
            format!("Base device init failed: {:?}", e).into()
        })?;

        // Initialize DMP (loads firmware)
        dmp.initialize()
            .map_err(|e| -> Box<EvalAltResult> { format!("DMP init failed: {:?}", e).into() })?;

        // Reset after firmware load so DMP starts cleanly.
        dmp.reset_dmp()
            .map_err(|e| format!("DMP reset failed: {:?}", e))?;
        dmp.reset_fifo()
            .map_err(|e| format!("FIFO reset failed: {:?}", e))?;

        // Set FIFO watermark lower to avoid overflow in polling/interrupt modes.
        dmp.set_fifo_watermark(256)
            .map_err(|e| format!("Set watermark failed: {:?}", e))?;

        // Enable FIFO and DMP after reset.
        dmp.dmp_fifo_enable(true)
            .map_err(|e| format!("FIFO enable failed: {:?}", e))?;
        dmp.enable_dmp(true)
            .map_err(|e| format!("DMP enable failed: {:?}", e))?;

        Ok(())
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
    /// icm.enable_sensor(mpu::Sensor::ACCELEROMETER, enable)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor_type: MpuSensor (use mpu::Sensor::* constants)
    /// - enable: bool
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:3
    #[rhai_fn(global, return_raw)]
    pub fn enable_sensor(
        icm: &mut RhaiIcm20948,
        sensor_type: MpuSensor,
        enable: bool,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        dmp.enable_dmp_sensor(sensor_type.0, enable)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Sets the output data rate (ODR) for a sensor.
    ///
    /// # Syntax
    /// icm.set_sensor_rate(mpu::OdrSensor::ODR_ACCEL, rate)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor_type: MpuOdrSensor (use mpu::OdrSensor::ODR_* constants)
    /// - rate: int (sample rate in Hz)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:3
    #[rhai_fn(global, return_raw)]
    pub fn set_sensor_rate(
        icm: &mut RhaiIcm20948,
        sensor_type: MpuOdrSensor,
        rate: i64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        dmp.set_sensor_rate(sensor_type.0, rate as i16)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Gets the output data rate (ODR) divider for a sensor.
    ///
    /// # Syntax
    /// icm.get_sensor_rate(mpu::OdrSensor::ODR_ACCEL)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor_type: MpuOdrSensor (use mpu::OdrSensor::ODR_* constants)
    /// # Returns
    /// int (divider value)
    /// # rhai-autodocs:index:4
    #[rhai_fn(global, return_raw)]
    pub fn get_sensor_rate(
        icm: &mut RhaiIcm20948,
        sensor_type: MpuOdrSensor,
    ) -> Result<i64, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let rate = dmp
            .get_sensor_rate(sensor_type.0)
            .map_err(|e| format!("{:?}", e))?;
        Ok(rate as i64)
    }

    /// Sets accelerometer full-scale range.
    ///
    /// # Syntax
    /// icm.set_accel_fullscale(mpu::AccelScale::FS_2G)
    /// # Parameters:
    /// - icm: Icm20948
    /// - scale: MpuAccelScale (use mpu::AccelScale::* constants)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:5
    #[rhai_fn(global, return_raw)]
    pub fn set_accel_fullscale(
        icm: &mut RhaiIcm20948,
        scale: MpuAccelScale,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let fs = match scale.0 {
            AccelFullScale::Fs2G => 2,
            AccelFullScale::Fs4G => 4,
            AccelFullScale::Fs8G => 8,
            AccelFullScale::Fs16G => 16,
        };
        dmp.set_accel_fsr(fs)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Gets accelerometer full-scale range.
    ///
    /// # Syntax
    /// icm.get_accel_fullscale()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// MpuAccelScale
    /// # rhai-autodocs:index:6
    #[rhai_fn(global, return_raw)]
    pub fn get_accel_fullscale(
        icm: &mut RhaiIcm20948,
    ) -> Result<MpuAccelScale, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let scale = dmp
            .device
            .get_accel_fullscale()
            .map_err(|e| format!("{:?}", e))?;
        Ok(MpuAccelScale(scale))
    }

    /// Sets gyroscope full-scale range.
    ///
    /// # Syntax
    /// icm.set_gyro_fullscale(mpu::GyroScale::FS_250DPS)
    /// # Parameters:
    /// - icm: Icm20948
    /// - scale: MpuGyroScale (use mpu::GyroScale::* constants)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:7
    #[rhai_fn(global, return_raw)]
    pub fn set_gyro_fullscale(
        icm: &mut RhaiIcm20948,
        scale: MpuGyroScale,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let fs = match scale.0 {
            GyroFullScale::Fs250Dps => 250,
            GyroFullScale::Fs500Dps => 500,
            GyroFullScale::Fs1000Dps => 1000,
            GyroFullScale::Fs2000Dps => 2000,
        };
        dmp.set_gyro_fsr(fs)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Gets gyroscope full-scale range.
    ///
    /// # Syntax
    /// icm.get_gyro_fullscale()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// MpuGyroScale
    /// # rhai-autodocs:index:8
    #[rhai_fn(global, return_raw)]
    pub fn get_gyro_fullscale(
        icm: &mut RhaiIcm20948,
    ) -> Result<MpuGyroScale, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let scale = dmp
            .device
            .get_gyro_fullscale()
            .map_err(|e| format!("{:?}", e))?;
        Ok(MpuGyroScale(scale))
    }

    /// Reads sensor temperature in Celsius.
    ///
    /// # Syntax
    /// icm.read_temperature()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// float (°C)
    /// # rhai-autodocs:index:9
    #[rhai_fn(global, return_raw)]
    pub fn read_temperature(icm: &mut RhaiIcm20948) -> Result<f64, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let temp = dmp
            .device
            .read_temperature()
            .map_err(|e| format!("{:?}", e))?;
        Ok(temp as f64)
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
    /// Map (latest cached values; includes any FIFO fields such as accel, gyro, quat, geomag, etc.)
    /// # rhai-autodocs:index:7
    #[rhai_fn(global, return_raw)]
    pub fn read_data(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let mut state = DmpFifoState::default();
        state.clear_data();

        match dmp.read_dmp_fifo_data(&mut state) {
            Ok(_) => {}
            Err(DmpFifoError::NoDataAvailable) => return Ok(get_last_data(icm)),
            Err(e) => return Err(format!("Error reading FIFO: {:?}", e).into()),
        }

        let map = fifo_state_to_map(&dmp, &state);

        if map.is_empty() {
            return Ok(get_last_data(icm));
        }

        Ok(merge_last_data_for_key(icm_cache_key(icm), &map))
    }

    /// Reads key DMP/INT status registers for debugging.
    ///
    /// # Syntax
    /// icm.debug_status()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map (int_status, dmp_int_status, int_pin_cfg, int_enable, user_ctrl, fifo_count,
    ///      data_out_ctl1, data_out_ctl2, data_intr_ctl, data_rdy_status, fifo_watermark)
    /// # rhai-autodocs:index:10
    #[rhai_fn(global, return_raw)]
    pub fn debug_status(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let int_status = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_STATUS)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let dmp_int_status = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::DMP_INT_STATUS)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let int_pin_cfg = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_PIN_CFG)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let int_enable = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_ENABLE)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let user_ctrl = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let fifo_count = dmp
            .device
            .get_fifo_count()
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;

        let data_out_ctl1 = read_mems_u16(
            &mut dmp,
            dmp::data_output_control::DATA_OUT_CTL1,
        )?;
        let data_out_ctl2 = read_mems_u16(
            &mut dmp,
            dmp::data_output_control::DATA_OUT_CTL2,
        )?;
        let data_intr_ctl = read_mems_u16(
            &mut dmp,
            dmp::data_output_control::DATA_INTR_CTL,
        )?;
        let data_rdy_status = read_mems_u16(
            &mut dmp,
            dmp::data_output_control::DATA_RDY_STATUS,
        )?;
        let fifo_watermark = read_mems_u16(
            &mut dmp,
            dmp::data_output_control::FIFO_WATERMARK,
        )?;

        let mut map = rhai::Map::new();
        map.insert("int_status".into(), (int_status as i64).into());
        map.insert("dmp_int_status".into(), (dmp_int_status as i64).into());
        map.insert("int_pin_cfg".into(), (int_pin_cfg as i64).into());
        map.insert("int_enable".into(), (int_enable as i64).into());
        map.insert("user_ctrl".into(), (user_ctrl as i64).into());
        map.insert("fifo_count".into(), (fifo_count as i64).into());
        map.insert("data_out_ctl1".into(), (data_out_ctl1 as i64).into());
        map.insert("data_out_ctl2".into(), (data_out_ctl2 as i64).into());
        map.insert("data_intr_ctl".into(), (data_intr_ctl as i64).into());
        map.insert("data_rdy_status".into(), (data_rdy_status as i64).into());
        map.insert("fifo_watermark".into(), (fifo_watermark as i64).into());

        Ok(map)
    }

    /// Reads INT_STATUS and DMP_INT_STATUS (also clears them if ANYRD_2CLEAR is enabled).
    ///
    /// # Syntax
    /// icm.read_int_status()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map (int_status, dmp_int_status, fifo_count)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn read_int_status(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let int_status = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::INT_STATUS)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let dmp_int_status = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::DMP_INT_STATUS)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let fifo_count = dmp
            .device
            .get_fifo_count()
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;

        let mut map = rhai::Map::new();
        map.insert("int_status".into(), (int_status as i64).into());
        map.insert("dmp_int_status".into(), (dmp_int_status as i64).into());
        map.insert("fifo_count".into(), (fifo_count as i64).into());

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

#[cfg(feature = "rhai")]
#[export_module]
pub mod icm20948_statics {
    /// Creates a new ICM20948 driver instance.
    ///
    /// # Syntax
    /// let icm = mpu.create(i2c_bus, address)
    /// # Parameters:
    /// - i2c_bus: string I2C bus path (e.g., "/dev/i2c-1")
    /// - address: int I2C address (0x68 or 0x69)
    /// # Returns
    /// Icm20948 object
    /// # rhai-autodocs:index:0
    #[rhai_fn(return_raw)]
    pub fn create(i2c_bus: &str, address: i64) -> Result<RhaiIcm20948, Box<EvalAltResult>> {
        let dev = I2cdev::new(i2c_bus)
            .map_err(|e| format!("Failed to open I2C bus {}: {}", i2c_bus, e))?;
        let delay = Delay;

        let icm = crate::new_i2c_device(dev, address as u8, delay);
        let dmp = DmpDriverIcm20948::new(icm);

        Ok(RhaiIcm20948::new(Arc::new(Mutex::new(dmp))))
    }
}
