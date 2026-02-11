#[cfg(feature = "rhai")]
use crate::dmp::DmpDriverIcm20948;
#[cfg(feature = "rhai")]
use crate::dmp_fifo::{DmpFifoError, DmpFifoState, Quaternion};
#[cfg(feature = "rhai")]
use crate::interface::I2cInterface;
#[cfg(feature = "rhai")]
use crate::geomag;
#[cfg(feature = "rhai")]
use crate::register::dmp;
#[cfg(feature = "rhai")]
use crate::register::registers::{bank0, bank1, bank2};
#[cfg(feature = "rhai")]
use crate::selftest;
#[cfg(feature = "rhai")]
use crate::types::{scale_factor, AccelFullScale, Activity, GyroFullScale, OdrSensor, Sensor};
#[cfg(feature = "rhai")]
use crate::Icm20948Error;
#[cfg(feature = "rhai")]
use crate::{compass::CompassType, conversion};
#[cfg(feature = "rhai")]
use linux_embedded_hal::{Delay, I2cdev};
#[cfg(feature = "rhai")]
use once_cell::sync::Lazy;
#[cfg(feature = "rhai")]
use rhai::plugin::*;
#[cfg(feature = "rhai")]
use rhai::{CustomType, EvalAltResult, TypeBuilder};
#[cfg(feature = "rhai")]
use std::collections::HashMap;
#[cfg(feature = "rhai")]
use std::fs::File;
#[cfg(feature = "rhai")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "rhai")]
const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;

#[cfg(feature = "rhai")]
pub type LinuxDmp = DmpDriverIcm20948<I2cInterface<I2cdev>, Delay>;

#[cfg(feature = "rhai")]
static LAST_MPU_DATA: Lazy<Mutex<HashMap<usize, rhai::Map>>> =
    Lazy::new(|| Mutex::new(HashMap::new()));
#[cfg(feature = "rhai")]
static MPU_SHARED_INSTANCES: Lazy<Mutex<HashMap<String, Arc<Mutex<LinuxDmp>>>>> =
    Lazy::new(|| Mutex::new(HashMap::new()));
#[cfg(feature = "rhai")]
static ACTIVE_MPU_MONITORS: Lazy<Mutex<HashMap<usize, usize>>> =
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
pub fn set_monitor_active_for_key(key: usize, active: bool) {
    if let Ok(mut guard) = ACTIVE_MPU_MONITORS.lock() {
        if active {
            let entry = guard.entry(key).or_insert(0);
            *entry = entry.saturating_add(1);
        } else if let Some(entry) = guard.get_mut(&key) {
            if *entry > 1 {
                *entry -= 1;
            } else {
                guard.remove(&key);
            }
        }
    }
}

#[cfg(feature = "rhai")]
fn has_active_monitor_for_key(key: usize) -> bool {
    ACTIVE_MPU_MONITORS
        .lock()
        .ok()
        .map(|guard| guard.contains_key(&key))
        .unwrap_or(false)
}

#[cfg(feature = "rhai")]
fn get_or_create_shared_icm(
    i2c_bus: &str,
    address: i64,
) -> Result<Arc<Mutex<LinuxDmp>>, Box<EvalAltResult>> {
    let key = format!("{}:{}", i2c_bus, address);
    let mut registry = MPU_SHARED_INSTANCES
        .lock()
        .map_err(|_| "MPU shared registry lock poisoned")?;

    if let Some(existing) = registry.get(&key) {
        return Ok(existing.clone());
    }

    let dev = I2cdev::new(i2c_bus)
        .map_err(|e| format!("Failed to open I2C bus {}: {}", i2c_bus, e))?;
    let delay = Delay;
    let icm = crate::new_i2c_device(dev, address as u8, delay);
    let dmp = DmpDriverIcm20948::new(icm);
    let shared = Arc::new(Mutex::new(dmp));
    registry.insert(key, shared.clone());
    Ok(shared)
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
fn read_mems_i16(dmp: &mut LinuxDmp, addr: u16) -> Result<i16, Box<EvalAltResult>> {
    let mut buf = [0u8; 2];
    dmp.device
        .read_mems(addr, &mut buf)
        .map_err(|e| format!("DMP mem read failed: {:?}", e))?;
    Ok(i16::from_be_bytes(buf))
}

#[cfg(feature = "rhai")]
fn read_mems_i32(dmp: &mut LinuxDmp, addr: u16) -> Result<i32, Box<EvalAltResult>> {
    let mut buf = [0u8; 4];
    dmp.device
        .read_mems(addr, &mut buf)
        .map_err(|e| format!("DMP mem read failed: {:?}", e))?;
    Ok(i32::from_be_bytes(buf))
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
fn vec3i16_to_map(values: [i16; 3]) -> rhai::Map {
    let mut map = rhai::Map::new();
    map.insert("x".into(), (values[0] as i64).into());
    map.insert("y".into(), (values[1] as i64).into());
    map.insert("z".into(), (values[2] as i64).into());
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
#[cfg(feature = "rhai")]
fn i32_array3_to_rhai(values: [i32; 3]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as i64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn convert_gyro_calibr_q20_to_rads(raw: [i32; 3]) -> [f32; 3] {
    // DMP calibrated gyro is in Q20, scaled to 2000 dps fullscale.
    let scale = 2000.0 / (1u32 << 20) as f32;
    [
        raw[0] as f32 * scale * DEG_TO_RAD,
        raw[1] as f32 * scale * DEG_TO_RAD,
        raw[2] as f32 * scale * DEG_TO_RAD,
    ]
}

#[cfg(feature = "rhai")]
fn convert_compass_calibr_q16_to_ut(raw: [i32; 3]) -> [f32; 3] {
    let scale = 1.0 / (1u32 << 16) as f32;
    [
        raw[0] as f32 * scale,
        raw[1] as f32 * scale,
        raw[2] as f32 * scale,
    ]
}

#[cfg(feature = "rhai")]
fn quat_to_gravity_g(quat: Quaternion) -> [f32; 3] {
    [
        2.0 * (quat.x * quat.z - quat.w * quat.y),
        2.0 * (quat.w * quat.x + quat.y * quat.z),
        quat.w * quat.w - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z,
    ]
}

#[cfg(feature = "rhai")]
fn apply_mount_matrix_vec(m: [i8; 9], v: [f32; 3]) -> [f32; 3] {
    [
        m[0] as f32 * v[0] + m[1] as f32 * v[1] + m[2] as f32 * v[2],
        m[3] as f32 * v[0] + m[4] as f32 * v[1] + m[5] as f32 * v[2],
        m[6] as f32 * v[0] + m[7] as f32 * v[1] + m[8] as f32 * v[2],
    ]
}

#[cfg(feature = "rhai")]
fn qconj(q: &Quaternion) -> Quaternion {
    Quaternion {
        w: q.w,
        x: -q.x,
        y: -q.y,
        z: -q.z,
        heading_accuracy_deg: None,
    }
}

#[cfg(feature = "rhai")]
fn qmul(a: &Quaternion, b: &Quaternion) -> Quaternion {
    Quaternion {
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        heading_accuracy_deg: a.heading_accuracy_deg,
    }
}

#[cfg(feature = "rhai")]
fn apply_mount_correction_quat(quat: Quaternion, mount: [i8; 9]) -> Quaternion {
    if let Some(q_mount) = crate::dmp_fifo::quaternion_from_mount_matrix(mount) {
        let qc = qconj(&q_mount);
        qmul(&quat, &qc)
    } else {
        quat
    }
}

#[cfg(feature = "rhai")]
fn quat_to_euler_raw(quaternion: &Quaternion) -> [f32; 3] {
    let w = quaternion.w;
    let x = quaternion.x;
    let y = quaternion.y;
    let z = quaternion.z;

    let m00 = 1.0 - 2.0 * (y * y + z * z);
    let m10 = 2.0 * (x * y + w * z);
    let m20 = 2.0 * (x * z - w * y);
    let m21 = 2.0 * (y * z + w * x);
    let m22 = 1.0 - 2.0 * (x * x + y * y);

    let yaw = (-m10).atan2(m00);
    let pitch = (-m21).atan2(m22);
    let roll = (m20).atan2((1.0 - m20 * m20).sqrt());

    let rad_to_deg = 180.0 / std::f32::consts::PI;
    [roll * rad_to_deg, pitch * rad_to_deg, yaw * rad_to_deg]
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
fn u8_array3_to_rhai(values: [u8; 3]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as i64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn i8_array9_to_rhai(values: [i8; 9]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as i64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn i16_array3_to_rhai(values: [i16; 3]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as i64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn f32_array3_to_rhai(values: [f32; 3]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push((value as f64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn q30_array9_to_rhai(values: [i32; 9]) -> rhai::Array {
    let mut array = rhai::Array::with_capacity(values.len());
    for value in values {
        array.push(((value as f64) / (1u64 << 30) as f64).into());
    }
    array
}

#[cfg(feature = "rhai")]
fn q30_from_f64(value: f64) -> Result<i32, Box<EvalAltResult>> {
    if !value.is_finite() {
        return Err("invalid soft-iron value".into());
    }
    let scaled = (value * (1u64 << 30) as f64).round();
    if scaled > i32::MAX as f64 || scaled < i32::MIN as f64 {
        return Err("soft-iron value out of range".into());
    }
    Ok(scaled as i32)
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
            if config.map(|cfg| cfg.calibration.scale == 0).unwrap_or(true) {
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
        let soft = dmp.device.base_state.soft_iron_matrix;
        let mut corrected = [0f32; 3];
        for i in 0..3 {
            let mut acc = 0f32;
            for j in 0..3 {
                acc += (soft[i * 3 + j] as f32) * out[j] / (1u64 << 30) as f32;
            }
            corrected[i] = acc;
        }
        corrected
    } else {
        [
            raw[0] as f32 * scale,
            raw[1] as f32 * scale,
            raw[2] as f32 * scale,
        ]
    }
}

#[cfg(feature = "rhai")]
fn convert_accel_raw_to_g(dmp: &LinuxDmp, raw: [i16; 3]) -> [f32; 3] {
    conversion::accel_raw_to_g(raw, dmp.device.base_state.accel_fullscale)
}

#[cfg(feature = "rhai")]
fn convert_gyro_raw_to_rads(dmp: &LinuxDmp, raw: [i16; 3]) -> [f32; 3] {
    let dps = conversion::gyro_raw_to_dps(raw, dmp.device.base_state.gyro_fullscale);
    [
        dps[0] * DEG_TO_RAD,
        dps[1] * DEG_TO_RAD,
        dps[2] * DEG_TO_RAD,
    ]
}

#[cfg(feature = "rhai")]
pub fn fifo_state_to_map(dmp: &LinuxDmp, state: &DmpFifoState) -> rhai::Map {
    let mut map = rhai::Map::new();
    let mut accel_g: Option<[f32; 3]> = None;
    let mut gravity_g: Option<[f32; 3]> = None;
    let mut compass_ut: Option<[f32; 3]> = None;
    let mount = dmp.device.base_state.mounting_matrix;
    let apply_mount = |v: [f32; 3]| -> [f32; 3] { apply_mount_matrix_vec(mount, v) };
    let apply_quat = |q: Quaternion| -> Quaternion { apply_mount_correction_quat(q, mount) };

    if let Some(accel) = state.accel_data {
        let g = apply_mount(convert_accel_raw_to_g(dmp, accel));
        accel_g = Some(g);
        map.insert("accel".into(), vec3f_to_map(g).into());
    }

    if let Some(gyro) = state.gyro_data {
        let g = apply_mount(convert_gyro_raw_to_rads(dmp, gyro));
        map.insert("gyro".into(), vec3f_to_map(g).into());
    }

    if let Some(compass) = state.compass_data {
        map.insert("compass_raw".into(), vec3i16_to_map(compass).into());
        let ut = apply_mount(convert_compass_raw_to_ut(dmp, compass));
        compass_ut = Some(ut);
    }

    if let Some(quat6) = state.quaternion6 {
        let q = apply_quat(quat6);
        map.insert("quat6".into(), quat_to_map(q).into());
    }

    if let Some(quat9) = state.quaternion9 {
        let q = apply_quat(quat9);
        map.insert("quat9".into(), quat_to_map(q).into());
    }

    if let Some(quatp6) = state.quaternionp6 {
        let q = apply_quat(quatp6);
        map.insert("quatp6".into(), quat_to_map(q).into());
    }

    if let Some(geomag) = state.geomag_data {
        let q = apply_quat(geomag);
        map.insert("geomag".into(), quat_to_map(q).into());
    }

    if let Some(gyro_calibr) = state.gyro_calibr {
        let g = apply_mount(convert_gyro_calibr_q20_to_rads(gyro_calibr));
        let mut gyro_map = vec3f_to_map(g);
        if let Some(accuracy) = state.gyro_accuracy {
            gyro_map.insert("accuracy".into(), (accuracy as i64).into());
        }
        map.insert("gyro_calibr".into(), gyro_map.into());
    }

    if let Some(compass_calibr) = state.compass_calibr {
        let ut = apply_mount(convert_compass_calibr_q16_to_ut(compass_calibr));
        compass_ut = Some(ut);
        let mut compass_map = vec3f_to_map(ut);
        if let Some(accuracy) = state.compass_accuracy {
            compass_map.insert("accuracy".into(), (accuracy as i64).into());
        }
        map.insert("compass_calibr".into(), compass_map.into());
    }

    if let Some(ut) = compass_ut {
        map.insert("compass".into(), vec3f_to_map(ut).into());
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

    if let Some(screen_rot) = state.screen_rotation {
        map.insert("screen_rotation".into(), (screen_rot as i64).into());
    }

    if let Some(activity) = state.activity_recognition {
        let mut act_map = rhai::Map::new();
        act_map.insert("start".into(), (activity.state_start as i64).into());
        act_map.insert("end".into(), (activity.state_end as i64).into());
        act_map.insert("time".into(), (activity.timestamp as i64).into());

        // Add decoded string
        let start_str = decode_activity(activity.state_start);
        let end_str = decode_activity(activity.state_end);
        act_map.insert("start_class".into(), start_str.into());
        act_map.insert("end_class".into(), end_str.into());

        map.insert("activity".into(), act_map.into());
    }

    if let Some(secondary) = state.secondary_on_off {
        map.insert("secondary_on_off".into(), (secondary as i64).into());
    }

    if let Some(quat) = state
        .quaternion9
        .or(state.quaternion6)
        .or(state.quaternionp6)
    {
        let q = apply_quat(quat);
        let g = quat_to_gravity_g(q);
        gravity_g = Some(g);
        map.insert("gravity".into(), vec3f_to_map(g).into());
    }

    if let (Some(acc), Some(gr)) = (accel_g, gravity_g) {
        let lin = [acc[0] - gr[0], acc[1] - gr[1], acc[2] - gr[2]];
        map.insert("linear_accel".into(), vec3f_to_map(lin).into());
    }

    if map.contains_key("quat9") {
        if let Some(value) = map.get("quat9").cloned() {
            map.insert("quat".into(), value);
        }
    } else if map.contains_key("quat6") {
        if let Some(value) = map.get("quat6").cloned() {
            map.insert("quat".into(), value);
        }
        if let Some(value) = map.get("quatp6").cloned() {
            map.insert("quat".into(), value);
        }
    }

    #[cfg(debug_assertions)]
    {
        // Debug print keys
        let mut keys_str = String::new();
        for key in map.keys() {
            keys_str.push_str(key.as_str());
            keys_str.push_str(", ");
        }
        log::debug!("MPU Map Keys: {}", keys_str);
    }

    map
}


fn decode_activity(val: u8) -> String {
    if val == 0 {
        return "None".to_string();
    }
    let mut s = String::new();
    let mut known_mask = 0u8;
    if val & 0x01 != 0 {
        s.push_str("Drive ");
        known_mask |= 0x01;
    }
    if val & 0x02 != 0 {
        s.push_str("Walk ");
        known_mask |= 0x02;
    }
    if val & 0x04 != 0 {
        s.push_str("Run ");
        known_mask |= 0x04;
    }
    if val & 0x08 != 0 {
        s.push_str("Bike ");
        known_mask |= 0x08;
    }
    if val & 0x10 != 0 {
        s.push_str("Tilt ");
        known_mask |= 0x10;
    }
    if val & 0x20 != 0 {
        s.push_str("Still ");
        known_mask |= 0x20;
    }
    let unknown = val & !known_mask;
    if unknown != 0 {
        s.push_str(&format!("Unknown(0x{:02X}) ", unknown));
    }
    if s.is_empty() {
        s.push_str("Unknown");
    }
    s.trim().to_string()
}

fn decode_activity_i64(val: i64) -> String {
    let byte = (val as u64 & 0xFF) as u8;
    decode_activity(byte)
}

fn decode_fp_rate(val: i64) -> (i64, String) {
    match val {
        0 => (56, "56Hz".to_string()),
        1 => (112, "112Hz".to_string()),
        3 => (225, "225Hz".to_string()),
        7 => (450, "450Hz".to_string()),
        15 => (900, "900Hz".to_string()),
        other => (-1, format!("Unknown({})", other)),
    }
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
        dmp.reapply_dmp_memory_config()
            .map_err(|e| format!("DMP config reapply failed: {:?}", e))?;
        dmp.device
            .low_power(false)
            .map_err(|e| format!("Disable low power failed: {:?}", e))?;
        dmp.device
            .force_low_noise()
            .map_err(|e| format!("Force low noise failed: {:?}", e))?;
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
        dmp.set_accel_fsr(fs).map_err(|e| format!("{:?}", e).into())
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
        dmp.set_gyro_fsr(fs).map_err(|e| format!("{:?}", e).into())
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
    pub fn get_gyro_fullscale(icm: &mut RhaiIcm20948) -> Result<MpuGyroScale, Box<EvalAltResult>> {
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

    /// Reads power and accel config registers for debugging (LP_EN, LP_CONFIG, ACCEL_CONFIG_1).
    ///
    /// # Syntax
    /// icm.debug_power_status()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map with register values
    /// # rhai-autodocs:index:9
    #[rhai_fn(global, return_raw)]
    pub fn debug_power_status(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let pwr_mgmt_1 = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::PWR_MGMT_1)
            .map_err(|e| format!("{:?}", e))?;
        let lp_config = dmp
            .device
            .read_mems_reg::<bank0::Bank>(bank0::LP_CONFIG)
            .map_err(|e| format!("{:?}", e))?;
        let accel_cfg = dmp
            .device
            .read_mems_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1)
            .map_err(|e| format!("{:?}", e))?;

        let mut map = rhai::Map::new();
        map.insert("pwr_mgmt_1".into(), (pwr_mgmt_1 as i64).into());
        map.insert("lp_config".into(), (lp_config as i64).into());
        map.insert("accel_config_1".into(), (accel_cfg as i64).into());
        map.insert(
            "wake_state".into(),
            (dmp.device.base_state.wake_state as i64).into(),
        );
        map.insert(
            "lp_en_support".into(),
            (dmp.device.base_state.lp_en_support as i64).into(),
        );
        Ok(map)
    }

    /// Reads corrected accelerometer data from hardware registers (g).
    ///
    /// # Syntax
    /// icm.read_accel_raw()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Array [x, y, z] (floats in g)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn read_accel_raw(icm: &mut RhaiIcm20948) -> Result<rhai::Array, Box<EvalAltResult>> {
        let mut dmp = icm
            .inner
            .try_lock()
            .map_err(|_| -> Box<EvalAltResult> { "MPU busy (accel read)".into() })?;
        let raw = dmp
            .device
            .accel_read_hw_reg_data()
            .map_err(|e| format!("{:?}", e))?;
        let mount = dmp.device.base_state.mounting_matrix;
        let g = apply_mount_matrix_vec(mount, convert_accel_raw_to_g(&dmp, raw));
        Ok(f32_array3_to_rhai(g))
    }

    /// Reads corrected magnetometer data (uT).
    ///
    /// # Syntax
    /// icm.read_compass_raw(timeout_ms)
    /// # Parameters:
    /// - icm: Icm20948
    /// - timeout_ms: int (0-1000 recommended)
    /// # Returns
    /// Array [x, y, z] (floats in uT)
    /// # rhai-autodocs:index:10
    #[rhai_fn(global, return_raw)]
    pub fn read_compass_raw(
        icm: &mut RhaiIcm20948,
        timeout_ms: i64,
    ) -> Result<rhai::Array, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let raw = dmp
            .device
            .read_compass_raw(timeout_ms.max(0) as u32)
            .map_err(|e| format!("{:?}", e))?;
        let mount = dmp.device.base_state.mounting_matrix;
        let ut = apply_mount_matrix_vec(mount, convert_compass_raw_to_ut(&dmp, raw));
        Ok(f32_array3_to_rhai(ut))
    }

    /// Reads hardware accel/gyro offsets from sensor registers.
    ///
    /// # Syntax
    /// icm.get_hw_offsets()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map { accel: [x,y,z], gyro: [x,y,z] }
    /// # rhai-autodocs:index:10
    #[rhai_fn(global, return_raw)]
    pub fn get_hw_offsets(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let accel = dmp
            .device
            .get_accel_hw_offsets()
            .map_err(|e| format!("{:?}", e))?;
        let gyro = dmp
            .device
            .get_gyro_hw_offsets()
            .map_err(|e| format!("{:?}", e))?;
        let mut map = rhai::Map::new();
        map.insert("accel".into(), i16_array3_to_rhai(accel).into());
        map.insert("gyro".into(), i16_array3_to_rhai(gyro).into());
        Ok(map)
    }

    /// Writes hardware accel/gyro offsets to sensor registers.
    ///
    /// # Syntax
    /// icm.set_hw_offsets(accel_offsets, gyro_offsets)
    /// # Parameters:
    /// - icm: Icm20948
    /// - accel_offsets: Array [x,y,z] (int16)
    /// - gyro_offsets: Array [x,y,z] (int16)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn set_hw_offsets(
        icm: &mut RhaiIcm20948,
        accel_offsets: rhai::Array,
        gyro_offsets: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if accel_offsets.len() != 3 || gyro_offsets.len() != 3 {
            return Err("Offsets must have length 3".into());
        }
        let accel = [
            accel_offsets[0]
                .as_int()
                .map_err(|_| "Invalid accel offset X")? as i16,
            accel_offsets[1]
                .as_int()
                .map_err(|_| "Invalid accel offset Y")? as i16,
            accel_offsets[2]
                .as_int()
                .map_err(|_| "Invalid accel offset Z")? as i16,
        ];
        let gyro = [
            gyro_offsets[0]
                .as_int()
                .map_err(|_| "Invalid gyro offset X")? as i16,
            gyro_offsets[1]
                .as_int()
                .map_err(|_| "Invalid gyro offset Y")? as i16,
            gyro_offsets[2]
                .as_int()
                .map_err(|_| "Invalid gyro offset Z")? as i16,
        ];
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.device
            .set_accel_hw_offsets(accel)
            .map_err(|e| format!("{:?}", e))?;
        dmp.device
            .set_gyro_hw_offsets(gyro)
            .map_err(|e| format!("{:?}", e))?;
        Ok(())
    }

    /// Sets the DMP B2S orientation matrix (row-major, i8 values).
    ///
    /// # Syntax
    /// icm.set_b2s_orientation_matrix(matrix)
    /// # Parameters:
    /// - icm: Icm20948
    /// - matrix: Array[9] (row-major, i8)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:12
    #[rhai_fn(global, return_raw)]
    pub fn set_b2s_orientation_matrix(
        icm: &mut RhaiIcm20948,
        matrix: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if matrix.len() != 9 {
            return Err("orientation matrix must have 9 elements".into());
        }
        let mut m = [0i8; 9];
        for (i, value) in matrix.into_iter().enumerate() {
            m[i] = value
                .as_int()
                .map_err(|_| "Invalid orientation matrix element")? as i8;
        }

        if !crate::dmp_fifo::is_rotation_matrix(m) {
            return Err("orientation matrix must be a proper rotation (det=+1)".into());
        }

        crate::dmp_fifo::set_mount_matrix(m);
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        dmp.device.base_state.mounting_matrix = m;
        if dmp.device.base_state.compass_config.is_some() {
            dmp.refresh_compass_matrix_from_config()
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        }

        // Set chip-to-body quaternion for DMP internal algorithms.
        if let Some(q) = crate::dmp_fifo::quaternion_from_mount_matrix(m) {
            let q30i = |v: f32| -> i32 { (v * (1i64 << 30) as f32) as i32 };
            let q_params = [q30i(q.w), q30i(q.x), q30i(q.y), q30i(q.z)];
            dmp.set_orientation_params(&q_params)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        }
        // Do not apply software correction; rely on DMP mounting configuration.
        // NOTE: The DMP's B2S matrix does NOT transform the quaternion output in FIFO.\n        // This is a known hardware limitation. We must use software correction instead.\n        crate::dmp_fifo::set_mount_quaternion_correction(true);

        // Re-initialize DMP algorithms so the new matrix takes effect
        dmp.reinit_dmp_algorithms()
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Sets the accelerometer/gyroscope scale/skew matrix (calibration).
    /// This matrix is multiplied with the mounting matrix before being applied to the DMP.
    ///
    /// # Syntax
    /// icm.set_accel_gyro_scale_matrix([s0, s1, s2, s3, s4, s5, s6, s7, s8])
    ///
    /// # Parameters:
    /// - icm: Icm20948
    /// - matrix: Array[9] (row-major, floats)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:13
    #[rhai_fn(global, return_raw)]
    pub fn set_accel_gyro_scale_matrix(
        icm: &mut RhaiIcm20948,
        matrix: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if matrix.len() != 9 {
            return Err("scale matrix must have 9 elements".into());
        }
        let mut m = [0f32; 9];
        for (i, value) in matrix.into_iter().enumerate() {
            m[i] = value
                .as_float()
                .map_err(|_| "Invalid scale matrix element")? as f32;
        }

        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.device.base_state.accel_gyro_scale_matrix = m;

        dmp.reinit_dmp_algorithms()
            .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;

        Ok(())
    }

    /// Gets the accelerometer/gyroscope scale/skew matrix.
    ///
    /// # Syntax
    /// icm.get_accel_gyro_scale_matrix()
    ///
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Array[9] (floats)
    /// # rhai-autodocs:index:14
    #[rhai_fn(global, return_raw)]
    pub fn get_accel_gyro_scale_matrix(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Array, Box<EvalAltResult>> {
        let dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let m = dmp.device.base_state.accel_gyro_scale_matrix;
        let mut result = rhai::Array::new();
        for val in m.iter() {
            result.push((*val as f64).into());
        }
        Ok(result)
    }

    /// Reads the current magnetometer calibration config.
    ///
    /// # Syntax
    /// icm.get_compass_calibration()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map { compass_type, scale, sensitivity[3], mounting_matrix[9], st_lower[3], st_upper[3], soft_iron_matrix[9] }
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn get_compass_calibration(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Map, Box<EvalAltResult>> {
        let dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let cfg = dmp
            .device
            .base_state
            .compass_config
            .as_ref()
            .ok_or_else(|| -> Box<EvalAltResult> { "Compass not configured".into() })?;

        let mut map = rhai::Map::new();
        map.insert(
            "compass_type".into(),
            format!("{:?}", cfg.compass_type).into(),
        );
        map.insert("scale".into(), (cfg.calibration.scale as i64).into());
        map.insert(
            "sensitivity".into(),
            u8_array3_to_rhai(cfg.calibration.sensitivity).into(),
        );
        map.insert(
            "mounting_matrix".into(),
            i8_array9_to_rhai(cfg.calibration.mounting_matrix).into(),
        );
        map.insert(
            "st_lower".into(),
            i16_array3_to_rhai(cfg.calibration.st_lower).into(),
        );
        map.insert(
            "st_upper".into(),
            i16_array3_to_rhai(cfg.calibration.st_upper).into(),
        );
        map.insert(
            "soft_iron_matrix".into(),
            q30_array9_to_rhai(dmp.device.base_state.soft_iron_matrix).into(),
        );
        Ok(map)
    }

    /// Reads the current compass soft-iron matrix (Q30) as floats.
    ///
    /// # Syntax
    /// icm.get_compass_soft_iron_matrix()
    /// # Returns
    /// Array[9] (row-major)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global)]
    pub fn get_compass_soft_iron_matrix(icm: &mut RhaiIcm20948) -> rhai::Array {
        if let Ok(dmp) = icm.inner.lock() {
            q30_array9_to_rhai(dmp.device.base_state.soft_iron_matrix)
        } else {
            rhai::Array::new()
        }
    }

    /// Reads the DMP compass matrix (CPASS_MTX) as raw Q30 integers.
    ///
    /// # Syntax
    /// icm.get_compass_dmp_matrix()
    /// # Returns
    /// Array[9] (row-major, i32)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn get_compass_dmp_matrix(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Array, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let addrs = [
            dmp::cpass::MTX_00,
            dmp::cpass::MTX_01,
            dmp::cpass::MTX_02,
            dmp::cpass::MTX_10,
            dmp::cpass::MTX_11,
            dmp::cpass::MTX_12,
            dmp::cpass::MTX_20,
            dmp::cpass::MTX_21,
            dmp::cpass::MTX_22,
        ];
        let mut out = rhai::Array::new();
        for addr in addrs {
            let v = read_mems_i32(&mut dmp, addr)?;
            out.push((v as i64).into());
        }
        Ok(out)
    }

    /// Sets the compass soft-iron matrix (row-major, floats).
    ///
    /// # Syntax
    /// icm.set_compass_soft_iron_matrix(matrix)
    /// # Parameters:
    /// - matrix: Array[9] (row-major)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn set_compass_soft_iron_matrix(
        icm: &mut RhaiIcm20948,
        matrix: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if matrix.len() != 9 {
            return Err("soft-iron matrix must have 9 elements".into());
        }
        let mut out = [0i32; 9];
        for (i, value) in matrix.into_iter().enumerate() {
            let v = value.as_float()?;
            out[i] = q30_from_f64(v)?;
        }
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.device.base_state.soft_iron_matrix = out;
        if dmp.device.base_state.compass_config.is_some() {
            dmp.refresh_compass_matrix_from_config()
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        }
        Ok(())
    }

    /// Sets a diagonal soft-iron matrix (scale only).
    ///
    /// # Syntax
    /// icm.set_compass_soft_iron_diag(sx, sy, sz)
    /// # Parameters:
    /// - sx: float
    /// - sy: float
    /// - sz: float
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn set_compass_soft_iron_diag(
        icm: &mut RhaiIcm20948,
        sx: f64,
        sy: f64,
        sz: f64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.device.base_state.soft_iron_matrix = [
            q30_from_f64(sx)?,
            0,
            0,
            0,
            q30_from_f64(sy)?,
            0,
            0,
            0,
            q30_from_f64(sz)?,
        ];
        if dmp.device.base_state.compass_config.is_some() {
            dmp.refresh_compass_matrix_from_config()
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        }
        Ok(())
    }

    /// Sets the compass orientation matrix (row-major, i8 values).
    ///
    /// # Syntax
    /// icm.set_compass_orientation_matrix(matrix)
    /// # Parameters:
    /// - icm: Icm20948
    /// - matrix: Array[9] (row-major, i8)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:11
    #[rhai_fn(global, return_raw)]
    pub fn set_compass_orientation_matrix(
        icm: &mut RhaiIcm20948,
        matrix: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if matrix.len() != 9 {
            return Err("compass orientation matrix must have 9 elements".into());
        }
        let mut m = [0i8; 9];
        for (i, value) in matrix.into_iter().enumerate() {
            m[i] = value
                .as_int()
                .map_err(|_| "Invalid compass orientation matrix element")?
                as i8;
        }

        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let cfg = dmp
            .device
            .base_state
            .compass_config
            .as_mut()
            .ok_or_else(|| -> Box<EvalAltResult> { "Compass not configured".into() })?;
        cfg.calibration.mounting_matrix = m;

        dmp.refresh_compass_matrix_from_config()
            .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        // Reset FIFO to ensure DMP applies the new compass matrix
        dmp.device
            .reset_fifo()
            .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })
    }

    /// Calibrates accelerometer and gyroscope.
    ///
    /// # Syntax
    /// icm.calibrate()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:5
    #[rhai_fn(global, return_raw)]
    pub fn calibrate(icm: &mut RhaiIcm20948) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        selftest::calibrate(&mut dmp.device, true)
            .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?;
        Ok(())
    }

    fn sync_hw_offsets_to_dmp_inner(dmp: &mut LinuxDmp) -> Result<rhai::Map, Icm20948Error> {
        let accel_hw = dmp.device.get_accel_hw_offsets()?;
        let gyro_hw = dmp.device.get_gyro_hw_offsets()?;

        // HW accel offsets are in 2048 LSB/g. DMP accel bias expects Q16 (65536/g).
        let accel_bias = [
            accel_hw[0] as i32 * 32,
            accel_hw[1] as i32 * 32,
            accel_hw[2] as i32 * 32,
        ];

        // HW gyro offsets are in 32.8 LSB/dps. DMP gyro bias expects Q20 at 2000 dps.
        // Q20 scale: 2^20 / 2000 = 524.288 LSB/dps -> factor 16 from 32.8 LSB/dps.
        let gyro_bias = [
            gyro_hw[0] as i32 * 16,
            gyro_hw[1] as i32 * 16,
            gyro_hw[2] as i32 * 16,
        ];

        dmp.set_bias_acc(&accel_bias)?;
        dmp.set_bias_gyr(&gyro_bias)?;

        let mut map = rhai::Map::new();
        map.insert("accel_hw".into(), i16_array3_to_rhai(accel_hw).into());
        map.insert("gyro_hw".into(), i16_array3_to_rhai(gyro_hw).into());
        map.insert("accel_bias".into(), i32_array3_to_rhai(accel_bias).into());
        map.insert("gyro_bias".into(), i32_array3_to_rhai(gyro_bias).into());
        Ok(map)
    }

    /// Sync HW accel/gyro offsets into DMP bias memory.
    ///
    /// # Syntax
    /// icm.sync_hw_offsets_to_dmp()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map { accel_hw, gyro_hw, accel_bias, gyro_bias }
    /// # rhai-autodocs:index:12
    #[rhai_fn(global, return_raw)]
    pub fn sync_hw_offsets_to_dmp(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        sync_hw_offsets_to_dmp_inner(&mut dmp).map_err(|e| format!("{:?}", e).into())
    }

    /// Reads data from the DMP FIFO.
    /// If an interrupt monitor is active for this device, returns cached values instead.
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
        let key = icm_cache_key(icm);
        if has_active_monitor_for_key(key) {
            return Ok(get_last_data(icm));
        }

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

        Ok(merge_last_data_for_key(key, &map))
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

        let data_out_ctl1 = read_mems_u16(&mut dmp, dmp::data_output_control::DATA_OUT_CTL1)?;
        let data_out_ctl2 = read_mems_u16(&mut dmp, dmp::data_output_control::DATA_OUT_CTL2)?;
        let data_intr_ctl = read_mems_u16(&mut dmp, dmp::data_output_control::DATA_INTR_CTL)?;
        let data_rdy_status = read_mems_u16(&mut dmp, dmp::data_output_control::DATA_RDY_STATUS)?;
        let fifo_watermark = read_mems_u16(&mut dmp, dmp::data_output_control::FIFO_WATERMARK)?;

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

    /// Reads gyro scale-related registers from HW and DMP for debugging.
    ///
    /// # Syntax
    /// icm.debug_gyro_scale()
    /// # Returns
    /// Map (gyro_config_1, gyro_smplrt_div, timebase_pll, dmp_gyro_fullscale, dmp_gyro_sf)
    /// # rhai-autodocs:index:10
    #[rhai_fn(global, return_raw)]
    pub fn debug_gyro_scale(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let gyro_config_1 = dmp
            .device
            .read_mems_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let gyro_smplrt_div = dmp
            .device
            .read_mems_reg::<bank2::Bank>(bank2::GYRO_SMPLRT_DIV)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;
        let timebase_pll = dmp
            .device
            .read_mems_reg::<bank1::Bank>(bank1::TIMEBASE_CORRECTION_PLL)
            .map_err(|e| Box::<EvalAltResult>::from(format!("{:?}", e)))?;

        let dmp_gyro_fullscale = read_mems_i32(&mut dmp, dmp::fsr::GYRO)?;
        let dmp_gyro_sf = read_mems_i32(&mut dmp, dmp::scale::GYRO_SF)?;

        let mut map = rhai::Map::new();
        map.insert("gyro_config_1".into(), (gyro_config_1 as i64).into());
        map.insert("gyro_smplrt_div".into(), (gyro_smplrt_div as i64).into());
        map.insert("timebase_pll".into(), (timebase_pll as i64).into());
        map.insert(
            "dmp_gyro_fullscale".into(),
            (dmp_gyro_fullscale as i64).into(),
        );
        map.insert("dmp_gyro_sf".into(), (dmp_gyro_sf as i64).into());

        Ok(map)
    }

    /// Overrides the DMP gyro scale factor (GYRO_SF) directly.
    ///
    /// # Syntax
    /// icm.set_dmp_gyro_sf(value)
    /// # Parameters:
    /// - icm: Icm20948
    /// - value: int (raw Q format scale factor)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:12
    #[rhai_fn(global, return_raw)]
    pub fn set_dmp_gyro_sf(
        icm: &mut RhaiIcm20948,
        value: i64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let bytes = (value as i32).to_be_bytes();
        dmp.device
            .write_mems(dmp::scale::GYRO_SF, &bytes)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Reads BAC (Basic Activity Classification) state variables from DMP memory.
    ///
    /// # Syntax
    /// icm.read_bac_states()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map with raw BAC state fields (ints)
    /// # rhai-autodocs:index:21
    #[rhai_fn(global, return_raw)]
    pub fn read_bac_states(icm: &mut RhaiIcm20948) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let mut map = rhai::Map::new();
        map.insert(
            "state".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STATE)? as i64).into(),
        );
        map.insert(
            "state_prev".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STATE_PREV)? as i64).into(),
        );
        map.insert(
            "act_on".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_ON)? as i64).into(),
        );
        map.insert(
            "act_off".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_OFF)? as i64).into(),
        );
        map.insert(
            "still_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STILL_S_F)? as i64).into(),
        );
        map.insert(
            "run_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::RUN_S_F)? as i64).into(),
        );
        map.insert(
            "drive_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::DRIVE_S_F)? as i64).into(),
        );
        map.insert(
            "walk_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::WALK_S_F)? as i64).into(),
        );
        map.insert(
            "smd_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::SMD_S_F)? as i64).into(),
        );
        map.insert(
            "bike_s_f".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::BIKE_S_F)? as i64).into(),
        );
        map.insert(
            "e1_short".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::E1_SHORT)? as i64).into(),
        );
        map.insert(
            "e2_short".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::E2_SHORT)? as i64).into(),
        );
        map.insert(
            "e3_short".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::E3_SHORT)? as i64).into(),
        );
        map.insert(
            "var_run".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::VAR_RUN)? as i64).into(),
        );
        map.insert(
            "drive_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::DRIVE_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "walk_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::WALK_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "smd_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::SMD_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "bike_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::BIKE_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "still_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STILL_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "run_confidence".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::RUN_CONFIDENCE)? as i64).into(),
        );
        map.insert(
            "mode_cntr".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::MODE_CNTR)? as i64).into(),
        );
        map.insert(
            "state_t_prev".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STATE_T_PREV)? as i64).into(),
        );
        map.insert(
            "act_t_on".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_T_ON)? as i64).into(),
        );
        map.insert(
            "act_t_off".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_T_OFF)? as i64).into(),
        );
        map.insert(
            "state_wrdbs_prev".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::STATE_WRDBS_PREV)? as i64).into(),
        );
        map.insert(
            "act_wrdbs_on".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_WRDBS_ON)? as i64).into(),
        );
        map.insert(
            "act_wrdbs_off".into(),
            (read_mems_i32(&mut dmp, dmp::bac_state::ACT_WRDBS_OFF)? as i64).into(),
        );
        map.insert(
            "act_on_off".into(),
            (read_mems_i16(&mut dmp, dmp::bac_state::ACT_ON_OFF)? as i64).into(),
        );
        map.insert(
            "prev_act_on_off".into(),
            (read_mems_i16(&mut dmp, dmp::bac_state::PREV_ACT_ON_OFF)? as i64).into(),
        );
        map.insert(
            "cntr".into(),
            (read_mems_i16(&mut dmp, dmp::bac_state::CNTR)? as i64).into(),
        );
        map.insert(
            "mag_on".into(),
            (read_mems_i16(&mut dmp, dmp::bac_state::MAG_ON)? as i64).into(),
        );
        map.insert(
            "ps_on".into(),
            (read_mems_i16(&mut dmp, dmp::bac_state::PS_ON)? as i64).into(),
        );

        Ok(map)
    }

    /// Reads BAC state variables and provides decoded activity labels.
    ///
    /// # Syntax
    /// icm.read_bac_states_pretty()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map { raw, decoded }
    /// # rhai-autodocs:index:22
    #[rhai_fn(global, return_raw)]
    pub fn read_bac_states_pretty(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Map, Box<EvalAltResult>> {
        let raw = read_bac_states(icm)?;

        let mut decoded = rhai::Map::new();
        let mut insert_decoded = |key: &str| {
            if let Some(val) = raw.get(key).and_then(|v| v.clone().try_cast::<i64>()) {
                decoded.insert(key.into(), decode_activity_i64(val).into());
            }
        };

        insert_decoded("state");
        insert_decoded("state_prev");
        insert_decoded("act_on");
        insert_decoded("act_off");
        insert_decoded("act_on_off");
        insert_decoded("prev_act_on_off");

        let mut map = rhai::Map::new();
        map.insert("raw".into(), raw.into());
        map.insert("decoded".into(), decoded.into());
        Ok(map)
    }

    /// Reads Flip/Pickup (tilt) configuration parameters from DMP memory.
    ///
    /// # Syntax
    /// icm.read_flip_pickup_params()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map with raw Flip/Pickup parameters (ints)
    /// # rhai-autodocs:index:23
    #[rhai_fn(global, return_raw)]
    pub fn read_flip_pickup_params(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Map, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let mut map = rhai::Map::new();

        map.insert(
            "var_alpha".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::VAR_ALPHA)? as i64).into(),
        );
        map.insert(
            "still_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::STILL_TH)? as i64).into(),
        );
        map.insert(
            "mid_still_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::MID_STILL_TH)? as i64).into(),
        );
        map.insert(
            "not_still_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::NOT_STILL_TH)? as i64).into(),
        );
        map.insert(
            "vib_rej_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::VIB_REJ_TH)? as i64).into(),
        );
        map.insert(
            "max_pickup_t_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::MAX_PICKUP_T_TH)? as i64).into(),
        );
        map.insert(
            "pickup_timeout_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::PICKUP_TIMEOUT_TH)? as i64).into(),
        );
        map.insert(
            "still_const_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::STILL_CONST_TH)? as i64).into(),
        );
        map.insert(
            "motion_const_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::MOTION_CONST_TH)? as i64).into(),
        );
        map.insert(
            "vib_count_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::VIB_COUNT_TH)? as i64).into(),
        );
        map.insert(
            "steady_tilt_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::STEADY_TILT_TH)? as i64).into(),
        );
        map.insert(
            "steady_tilt_up_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::STEADY_TILT_UP_TH)? as i64).into(),
        );
        map.insert(
            "z_flat_th_minus".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::Z_FLAT_TH_MINUS)? as i64).into(),
        );
        map.insert(
            "z_flat_th_plus".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::Z_FLAT_TH_PLUS)? as i64).into(),
        );
        map.insert(
            "dev_in_pocket_th".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::DEV_IN_POCKET_TH)? as i64).into(),
        );
        map.insert(
            "pickup_cntr".into(),
            (read_mems_i16(&mut dmp, dmp::flip_pickup::PICKUP_CNTR)? as i64).into(),
        );
        map.insert(
            "rate".into(),
            (read_mems_i32(&mut dmp, dmp::flip_pickup::RATE)? as i64).into(),
        );

        Ok(map)
    }

    /// Reads Flip/Pickup (tilt) parameters with decoded rate info.
    ///
    /// # Syntax
    /// icm.read_flip_pickup_params_pretty()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// Map { raw, decoded }
    /// # rhai-autodocs:index:24
    #[rhai_fn(global, return_raw)]
    pub fn read_flip_pickup_params_pretty(
        icm: &mut RhaiIcm20948,
    ) -> Result<rhai::Map, Box<EvalAltResult>> {
        let raw = read_flip_pickup_params(icm)?;
        let mut decoded = rhai::Map::new();

        if let Some(rate) = raw.get("rate").and_then(|v| v.clone().try_cast::<i64>()) {
            let (hz, label) = decode_fp_rate(rate);
            decoded.insert("rate_hz".into(), hz.into());
            decoded.insert("rate_label".into(), label.into());
        }

        let mut map = rhai::Map::new();
        map.insert("raw".into(), raw.into());
        map.insert("decoded".into(), decoded.into());
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

    /// Gets the current pedometer step count.
    ///
    /// # Syntax
    /// icm.get_pedometer_steps()
    /// # Parameters:
    /// - icm: Icm20948
    /// # Returns
    /// int (number of steps)
    /// # rhai-autodocs:index:12
    #[rhai_fn(global, return_raw)]
    pub fn get_pedometer_steps(icm: &mut RhaiIcm20948) -> Result<i64, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let steps = dmp
            .get_pedometer_num_of_steps()
            .map_err(|e| format!("{:?}", e))?;
        Ok(steps as i64)
    }

    /// Enables or disables Wake-on-Motion (WOM).
    ///
    /// # Syntax
    /// icm.enable_wom(enable)
    /// # Parameters:
    /// - icm: Icm20948
    /// - enable: bool
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:13
    #[rhai_fn(global, return_raw)]
    pub fn enable_wom(icm: &mut RhaiIcm20948, enable: bool) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        dmp.enable_hardware_wom(enable)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Sets the Wake-on-Motion (WOM) threshold.
    ///
    /// # Syntax
    /// icm.set_wom_threshold(threshold)
    /// # Parameters:
    /// - icm: Icm20948
    /// - threshold: int
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:14
    #[rhai_fn(global, return_raw)]
    pub fn set_wom_threshold(
        icm: &mut RhaiIcm20948,
        threshold: i64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let val = threshold.clamp(0, 255) as u8;
        dmp.set_hardware_wom_threshold(val)
            .map_err(|e| format!("{:?}", e).into())
    }

    /// Gets the bias for a specific sensor (Accel, Gyro, or Compass).
    ///
    /// # Syntax
    /// icm.get_bias(mpu::Sensor::ACCELEROMETER)
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor: MpuSensor
    /// # Returns
    /// Array [x, y, z] (integers)
    /// # rhai-autodocs:index:15
    #[rhai_fn(global, return_raw)]
    pub fn get_bias(
        icm: &mut RhaiIcm20948,
        sensor: MpuSensor,
    ) -> Result<rhai::Array, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        let bias = match sensor.0 {
            Sensor::Accelerometer => dmp.get_bias_acc(),
            Sensor::Gyroscope => dmp.get_bias_gyr(),
            Sensor::GeomagneticField => dmp.get_bias_cmp(),
            _ => {
                return Err(format!(
                    "Sensor {:?} does not have directly accessible bias",
                    sensor.0
                )
                .into())
            }
        }
        .map_err(|e| format!("{:?}", e))?;

        let mut arr = rhai::Array::new();
        arr.push((bias[0] as i64).into());
        arr.push((bias[1] as i64).into());
        arr.push((bias[2] as i64).into());
        Ok(arr)
    }

    /// Sets the bias for a specific sensor (Accel, Gyro, or Compass).
    ///
    /// # Syntax
    /// icm.set_bias(mpu::Sensor::ACCELEROMETER, [x, y, z])
    /// # Parameters:
    /// - icm: Icm20948
    /// - sensor: MpuSensor
    /// - bias: Array [x, y, z] (integers)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:16
    #[rhai_fn(global, return_raw)]
    pub fn set_bias(
        icm: &mut RhaiIcm20948,
        sensor: MpuSensor,
        bias_arr: rhai::Array,
    ) -> Result<(), Box<EvalAltResult>> {
        if bias_arr.len() != 3 {
            return Err("Bias array must have exactly 3 elements".into());
        }
        let x = bias_arr[0].as_int().map_err(|_| "Invalid bias X type")? as i32;
        let y = bias_arr[1].as_int().map_err(|_| "Invalid bias Y type")? as i32;
        let z = bias_arr[2].as_int().map_err(|_| "Invalid bias Z type")? as i32;
        let bias = [x, y, z];

        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;
        match sensor.0 {
            Sensor::Accelerometer => dmp.set_bias_acc(&bias),
            Sensor::Gyroscope => dmp.set_bias_gyr(&bias),
            Sensor::GeomagneticField => dmp.set_bias_cmp(&bias),
            _ => {
                return Err(
                    format!("Sensor {:?} does not have directly settable bias", sensor.0).into(),
                )
            }
        }
        .map_err(|e| format!("{:?}", e).into())
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
    /// Icm20948 object (shared per bus/address)
    /// # rhai-autodocs:index:0
    #[rhai_fn(return_raw)]
    pub fn create(i2c_bus: &str, address: i64) -> Result<RhaiIcm20948, Box<EvalAltResult>> {
        Ok(RhaiIcm20948::new(get_or_create_shared_icm(
            i2c_bus, address,
        )?))
    }

    /// Returns a shared ICM20948 driver instance without reinitializing it.
    ///
    /// # Syntax
    /// let icm = mpu.get_shared(i2c_bus, address)
    /// # Parameters:
    /// - i2c_bus: string I2C bus path (e.g., "/dev/i2c-1")
    /// - address: int I2C address (0x68 or 0x69)
    /// # Returns
    /// Icm20948 object (shared per bus/address)
    /// # rhai-autodocs:index:1
    #[rhai_fn(return_raw, name = "get_shared")]
    pub fn get_shared(i2c_bus: &str, address: i64) -> Result<RhaiIcm20948, Box<EvalAltResult>> {
        Ok(RhaiIcm20948::new(get_or_create_shared_icm(
            i2c_bus, address,
        )?))
    }
    /// Writes a raw register value.
    ///
    /// # Syntax
    /// icm.write_register(bank, reg, value)
    /// # Parameters:
    /// - icm: Icm20948
    /// - bank: int (0-3)
    /// - reg: int (0-255)
    /// - value: int (0-255)
    /// # Returns
    /// Result (Empty if success)
    /// # rhai-autodocs:index:20
    #[rhai_fn(global, return_raw)]
    pub fn write_register(
        icm: &mut RhaiIcm20948,
        bank: i64,
        reg: i64,
        value: i64,
    ) -> Result<(), Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        // Manual bank selection logic is hidden in device methods,
        // but we can implementation a generic write helper in dmp/device or
        // map bank integer to types.
        // For now, simpler to map bank index to Bank type.

        // Unfortunately `write_mems_reg` takes a generic type param.
        // We can use a match.
        match bank {
            0 => dmp
                .device
                .write_mems_reg::<crate::register::registers::bank0::Bank>(reg as u8, value as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() }),
            1 => dmp
                .device
                .write_mems_reg::<crate::register::registers::bank1::Bank>(reg as u8, value as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() }),
            2 => dmp
                .device
                .write_mems_reg::<crate::register::registers::bank2::Bank>(reg as u8, value as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() }),
            3 => dmp
                .device
                .write_mems_reg::<crate::register::registers::bank3::Bank>(reg as u8, value as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() }),
            _ => Err("Invalid bank index (0-3)".into()),
        }
    }

    /// Reads a raw register value.
    ///
    /// # Syntax
    /// icm.read_register(bank, reg)
    /// # Parameters:
    /// - icm: Icm20948
    /// - bank: int (0-3)
    /// - reg: int (0-255)
    /// # Returns
    /// int (value)
    /// # rhai-autodocs:index:21
    #[rhai_fn(global, return_raw)]
    pub fn read_register(
        icm: &mut RhaiIcm20948,
        bank: i64,
        reg: i64,
    ) -> Result<i64, Box<EvalAltResult>> {
        let mut dmp = icm.inner.lock().map_err(|e| e.to_string())?;

        let val = match bank {
            0 => dmp
                .device
                .read_mems_reg::<crate::register::registers::bank0::Bank>(reg as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?,
            1 => dmp
                .device
                .read_mems_reg::<crate::register::registers::bank1::Bank>(reg as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?,
            2 => dmp
                .device
                .read_mems_reg::<crate::register::registers::bank2::Bank>(reg as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?,
            3 => dmp
                .device
                .read_mems_reg::<crate::register::registers::bank3::Bank>(reg as u8)
                .map_err(|e| -> Box<EvalAltResult> { format!("{:?}", e).into() })?,
            _ => return Err("Invalid bank index (0-3)".into()),
        };
        Ok(val as i64)
    }

    /// Computes atan2(y, x).
    ///
    /// # Syntax
    /// atan2(y, x)
    /// # Parameters:
    /// - y: float
    /// - x: float
    /// # Returns
    /// float (radians)
    /// # rhai-autodocs:index:22
    #[rhai_fn(global)]
    pub fn atan2(y: f64, x: f64) -> f64 {
        y.atan2(x)
    }

    /// Computes magnetic declination using WMM2025.
    ///
    /// # Syntax
    /// mpu.magnetic_declination(lat, lon, year, month, day)
    /// # Parameters:
    /// - lat: float (degrees)
    /// - lon: float (degrees)
    /// - year: int
    /// - month: int (1-12)
    /// - day: int (1-31)
    /// # Returns
    /// float (degrees, positive east)
    /// # rhai-autodocs:index:22
    #[rhai_fn(name = "magnetic_declination", global, return_raw)]
    pub fn magnetic_declination(
        lat: f64,
        lon: f64,
        year: i64,
        month: i64,
        day: i64,
    ) -> Result<f64, Box<EvalAltResult>> {
        if month < 1 || month > 12 {
            return Err("Invalid month".into());
        }
        if day < 1 || day > 31 {
            return Err("Invalid day".into());
        }
        geomag::magnetic_declination(
            lat,
            lon,
            0.0,
            year as i32,
            month as u32,
            day as u32,
        )
        .map_err(|e| e.into())
    }

    /// Computes magnetic declination using WMM2025 (with altitude).
    ///
    /// # Syntax
    /// mpu.magnetic_declination(lat, lon, alt_m, year, month, day)
    /// # Parameters:
    /// - lat: float (degrees)
    /// - lon: float (degrees)
    /// - alt_m: float (meters above ellipsoid)
    /// - year: int
    /// - month: int (1-12)
    /// - day: int (1-31)
    /// # Returns
    /// float (degrees, positive east)
    /// # rhai-autodocs:index:22
    #[rhai_fn(name = "magnetic_declination", global, return_raw)]
    pub fn magnetic_declination_alt(
        lat: f64,
        lon: f64,
        alt_m: f64,
        year: i64,
        month: i64,
        day: i64,
    ) -> Result<f64, Box<EvalAltResult>> {
        if month < 1 || month > 12 {
            return Err("Invalid month".into());
        }
        if day < 1 || day > 31 {
            return Err("Invalid day".into());
        }
        geomag::magnetic_declination(
            lat,
            lon,
            alt_m,
            year as i32,
            month as u32,
            day as u32,
        )
        .map_err(|e| e.into())
    }

    /// Convert quaternion to Euler angles (degrees).
    ///
    /// # Syntax
    /// mpu.quat_to_euler([w, x, y, z])
    /// mpu.quat_to_euler(#{ w: , x: , y: , z: })
    /// # Parameters:
    /// - quat: Array [w, x, y, z] (floats)
    /// # Returns
    /// Array [roll, pitch, yaw] (floats in degrees)
    /// # rhai-autodocs:index:22
    #[rhai_fn(global, return_raw)]
pub fn quat_to_euler(quat: rhai::Array) -> Result<rhai::Array, Box<EvalAltResult>> {
        if quat.len() != 4 {
            return Err("Quaternion array must have exactly 4 elements".into());
        }
        let w = quat[0]
            .as_float()
            .map_err(|_| "Invalid quaternion W type")? as f32;
        let x = quat[1]
            .as_float()
            .map_err(|_| "Invalid quaternion X type")? as f32;
        let y = quat[2]
            .as_float()
            .map_err(|_| "Invalid quaternion Y type")? as f32;
        let z = quat[3]
            .as_float()
            .map_err(|_| "Invalid quaternion Z type")? as f32;
        let quaternion = Quaternion {
            w,
            x,
            y,
            z,
            heading_accuracy_deg: None,
        };
        let angles = quat_to_euler_raw(&quaternion);
        let mut result = rhai::Array::new();
        result.push((angles[0] as f64).into());
        result.push((angles[1] as f64).into());
        result.push((angles[2] as f64).into());
        Ok(result)
    }

    #[rhai_fn(name = "quat_to_euler", global, return_raw)]
pub fn quat_to_euler_map(quat: rhai::Map) -> Result<rhai::Map, Box<EvalAltResult>> {
        let w = quat
            .get("w")
            .ok_or("Missing quaternion W")?
            .as_float()
            .map_err(|_| "Invalid quaternion W type")? as f32;
        let x = quat
            .get("x")
            .ok_or("Missing quaternion X")?
            .as_float()
            .map_err(|_| "Invalid quaternion X type")? as f32;
        let y = quat
            .get("y")
            .ok_or("Missing quaternion Y")?
            .as_float()
            .map_err(|_| "Invalid quaternion Y type")? as f32;
        let z = quat
            .get("z")
            .ok_or("Missing quaternion Z")?
            .as_float()
            .map_err(|_| "Invalid quaternion Z type")? as f32;
        let quaternion = Quaternion {
            w,
            x,
            y,
            z,
            heading_accuracy_deg: None,
        };
        let angles = quat_to_euler_raw(&quaternion);
        let mut result = rhai::Map::new();
        result.insert("roll".into(), (angles[0] as f64).into());
        result.insert("pitch".into(), (angles[1] as f64).into());
        result.insert("yaw".into(), (angles[2] as f64).into());
        Ok(result)
    }

    /// Converts Euler angles (degrees) to a quaternion.
    /// ///
    /// # Syntax
    /// mpu.euler_to_quat(roll, pitch, yaw)
    /// mpu.euler_to_quat(#{ roll: , pitch: , yaw: })
    /// # Parameters:
    /// - roll: float (degrees)
    /// - pitch: float (degrees)
    /// - yaw: float (degrees)
    /// # Returns
    /// Array [w, x, y, z] (floats)
    /// # rhai-autodocs:index:23
    #[rhai_fn(global, return_raw)]
    pub fn euler_to_quat(
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) -> Result<rhai::Array, Box<EvalAltResult>> {
        let quaternion =
            crate::dmp_fifo::euler_to_quaternion(roll as f32, pitch as f32, yaw as f32);
        let mut result = rhai::Array::new();
        result.push((quaternion.w as f64).into());
        result.push((quaternion.x as f64).into());
        result.push((quaternion.y as f64).into());
        result.push((quaternion.z as f64).into());
        Ok(result)
    }

    #[rhai_fn(name = "euler_to_quat", global, return_raw)]
    pub fn euler_to_quat_map(angles: rhai::Map) -> Result<rhai::Map, Box<EvalAltResult>> {
        let roll = angles
            .get("roll")
            .ok_or("Missing roll angle")?
            .as_float()
            .map_err(|_| "Invalid roll angle type")? as f32;
        let pitch = angles
            .get("pitch")
            .ok_or("Missing pitch angle")?
            .as_float()
            .map_err(|_| "Invalid pitch angle type")? as f32;
        let yaw = angles
            .get("yaw")
            .ok_or("Missing yaw angle")?
            .as_float()
            .map_err(|_| "Invalid yaw angle type")? as f32;
        let quaternion = crate::dmp_fifo::euler_to_quaternion(roll, pitch, yaw);
        let mut result = rhai::Map::new();
        result.insert("w".into(), (quaternion.w as f64).into());
        result.insert("x".into(), (quaternion.x as f64).into());
        result.insert("y".into(), (quaternion.y as f64).into());
        result.insert("z".into(), (quaternion.z as f64).into());
        Ok(result)
    }
}
