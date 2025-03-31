//! DMP implementation for ICM20948

use crate::device::{Icm20948, Icm20948Error};
use crate::firmware;
use crate::types::{GyroFullScale, AccelFullScale};
use crate::register::dmp;
use crate::interface::Interface;
use embedded_hal::blocking::delay::DelayMs;

/// DMP sensor types
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Sensor {
    /// Accelerometer
    Accelerometer = 0,
    /// Gyroscope
    Gyroscope = 1,
    /// LPQ
    LPQ = 2,
    /// Compass
    Compass = 3,
    /// ALS
    ALS = 4,
    /// SixQ
    SixQ = 5,
    /// NineQ
    NineQ = 6,
    /// GeoMag
    GeoMag = 7,
    /// PedQ
    PedQ = 8,
    /// Pressure
    Pressure = 9,
    /// CalibGyro
    CalibGyro = 10,
    /// CalibCompass
    CalibCompass = 11,
    /// StepCounter
    StepCounter = 12,
    /// ActivityClassifier
    ActivityClassifier = 13,
    /// FlipPickup
    FlipPickup = 14,
    /// BringToSee
    BringToSee = 15,
    // Additional sensors
    /// SixQAccel
    SixQAccel = 16,
    /// NineQAccel
    NineQAccel = 17,
    /// GeoMagCpass
    GeoMagCpass = 18,
    /// NineQCpass
    NineQCpass = 19,
    
    // Wakeup versions
    /// WakeupAccel
    WakeupAccel = 20,
    /// WakeupGyro
    WakeupGyro = 21,
    /// WakeupCompass
    WakeupCompass = 22,
    /// WakeupAls
    WakeupAls = 23,
    /// WakeupSixQ
    WakeupSixQ = 24,
    /// WakeupNineQ
    WakeupNineQ = 25,
    /// WakeupGeoMag
    WakeupGeoMag = 26,
    /// WakeupPedQ
    WakeupPedQ = 27,
    /// WakeupPressure
    WakeupPressure = 28,
    /// WakeupCalibGyro
    WakeupCalibGyro = 29,
    /// WakeupCalibCompass
    WakeupCalibCompass = 30,
    /// WakeupStepCounter
    WakeupStepCounter = 31,
    /// WakeupTiltDetector
    WakeupTiltDetector = 32,
    
    // Additional wakeup sensors
    /// WakeupSixQAccel
    WakeupSixQAccel = 33,
    /// WakeupNineQAccel
    WakeupNineQAccel = 34,
    /// WakeupGeoMagCpass
    WakeupGeoMagCpass = 35,
    /// WakeupNineQCpass
    WakeupNineQCpass = 36,
    
    /// InvalidSensor
    InvalidSensor = 255,
}

/// Algorithm frequency options for DMP
#[derive(Debug, Clone, Copy)]
#[repr(u16)]
pub enum DmpAlgoFreq {
    /// Freq56Hz
    Freq56Hz = 56,
    /// Freq112Hz
    Freq112Hz = 112,
    /// Freq225Hz
    Freq225Hz = 225,
    /// Freq450Hz
    Freq450Hz = 450,
    /// Freq900Hz
    Freq900Hz = 900,
}

/// Parameters for accel calibration
#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub enum AccelCalParam {
    /// AlphaVar
    AlphaVar = 0,
    /// AVar
    AVar = 1,
    /// Div
    Div = 2,
}

/// Parameters for compass calibration
#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub enum CompassCalParam {
    /// TimeBuffer
    TimeBuffer = 0,
    /// Radius3dThreshAnomaly
    Radius3dThreshAnomaly = 1,
}

/// Output mask bits for data_output_control1
#[allow(non_upper_case_globals)]
pub mod output_mask {
    /// ACCEL_SET
    pub const ACCEL_SET: u16 = 0x8000;
    /// GYRO_SET
    pub const GYRO_SET: u16 = 0x4000;
    /// CPASS_SET
    pub const CPASS_SET: u16 = 0x2000;
    /// ALS_SET
    pub const ALS_SET: u16 = 0x1000;
    /// QUAT6_SET
    pub const QUAT6_SET: u16 = 0x0800;
    /// QUAT9_SET
    pub const QUAT9_SET: u16 = 0x0400;
    /// PQUAT6_SET
    pub const PQUAT6_SET: u16 = 0x0200;
    /// GEOMAG_SET
    pub const GEOMAG_SET: u16 = 0x0100;
    /// PRESSURE_SET
    pub const PRESSURE_SET: u16 = 0x0080;
    /// GYRO_CALIBR_SET
    pub const GYRO_CALIBR_SET: u16 = 0x0040;
    /// CPASS_CALIBR_SET
    pub const CPASS_CALIBR_SET: u16 = 0x0020;
    /// PED_STEPDET_SET
    pub const PED_STEPDET_SET: u16 = 0x0010;
    /// HEADER2_SET
    pub const HEADER2_SET: u16 = 0x0008;
    /// PED_STEPIND_SET
    pub const PED_STEPIND_SET: u16 = 0x0007;
}

/// Output mask bits for data_output_control2
#[allow(non_upper_case_globals)]
pub mod output_mask2 {
    /// ACCEL_ACCURACY_SET
    pub const ACCEL_ACCURACY_SET: u16 = 0x4000;
    /// GYRO_ACCURACY_SET
    pub const GYRO_ACCURACY_SET: u16 = 0x2000;
    /// CPASS_ACCURACY_SET
    pub const CPASS_ACCURACY_SET: u16 = 0x1000;
    /// BATCH_MODE_EN
    pub const BATCH_MODE_EN: u16 = 0x0100;
}

/// Motion event control mask bits
#[allow(non_upper_case_globals)]
pub mod motion_event_control {
    /// BAC_WEAR_EN
    pub const BAC_WEAR_EN: u16 = 0x8000;
    /// PEDOMETER_EN
    pub const PEDOMETER_EN: u16 = 0x4000;
    /// PEDOMETER_INT_EN
    pub const PEDOMETER_INT_EN: u16 = 0x2000;
    /// SMD_EN
    pub const SMD_EN: u16 = 0x0800;
    /// ACCEL_CAL_EN
    pub const ACCEL_CAL_EN: u16 = 0x0200;
    /// GYRO_CAL_EN
    pub const GYRO_CAL_EN: u16 = 0x0100;
    /// COMPASS_CAL_EN
    pub const COMPASS_CAL_EN: u16 = 0x0080;
    /// NINE_AXIS_EN
    pub const NINE_AXIS_EN: u16 = 0x0040;
    /// GEOMAG_EN
    pub const GEOMAG_EN: u16 = 0x0008;
    /// BTS_LTS_EN
    pub const BTS_LTS_EN: u16 = 0x0004;
    /// BAC_ACCEL_ONLY_EN
    pub const BAC_ACCEL_ONLY_EN: u16 = 0x0002;
}

/// Driver para el DMP (Digital Motion Processor) del ICM20948
pub struct DmpDriverIcm20948<I, D> {
    device: Icm20948<I, D>,
}

impl<I, D, E> DmpDriverIcm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    // Icm20948Error: From<E>,
{
    /// Creates a new `DmpDriverIcm20948`.
    pub fn new(device: Icm20948<I, D>) -> Self {
        Self { device }
    }

    /// Verificar si el firmware está cargado
    pub fn is_firmware_loaded(&self) -> bool {
        self.device.is_firmware_loaded()
    }
    
    /// Load DMP firmware
    pub fn load_firmware(&mut self, firmware_data: &[u8]) -> Result<(), Icm20948Error> {
        firmware::load_firmware(&mut self.device, firmware_data, firmware::DMP_LOAD_START)
    }

    /// Get DMP start address
    pub fn get_dmp_start_address(&mut self) -> u16 {
        firmware::get_dmp_start_address()
    }

    /// Reset DMP control registers
    pub fn reset_control_registers(&mut self) -> Result<(), Icm20948Error> {
        // Reset all control registers to zero
        let zeros = [0u8, 0u8];
        
        self.device.write_mems(dmp::DATA_OUT_CTL1, &zeros)?;
        self.device.write_mems(dmp::DATA_OUT_CTL2, &zeros)?;
        self.device.write_mems(dmp::DATA_INTR_CTL, &zeros)?;
        self.device.write_mems(dmp::MOTION_EVENT_CTL, &zeros)?;
        self.device.write_mems(dmp::DATA_RDY_STATUS, &zeros)?;
        
        Ok(())
    }
    
    /// Set data output control register 1
    pub fn set_data_output_control1(&mut self, output_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = output_mask.to_be_bytes();
        self.device.write_mems(dmp::DATA_OUT_CTL1, &bytes)
    }
    
    /// Set data output control register 2
    pub fn set_data_output_control2(&mut self, output_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = output_mask.to_be_bytes();
        self.device.write_mems(dmp::DATA_OUT_CTL2, &bytes)
    }
    
    /// Set data interrupt control register
    pub fn set_data_interrupt_control(&mut self, interrupt_ctl: u32) -> Result<(), Icm20948Error> {
        // Only the lower 16 bits are used
        let bytes = (interrupt_ctl as u16).to_be_bytes();
        self.device.write_mems(dmp::DATA_INTR_CTL, &bytes)
    }
    
    /// Set FIFO watermark
    pub fn set_fifo_watermark(&mut self, fifo_wm: u16) -> Result<(), Icm20948Error> {
        let bytes = fifo_wm.to_be_bytes();
        self.device.write_mems(dmp::FIFO_WATERMARK, &bytes)
    }
    
    /// Set data ready status register
    pub fn set_data_rdy_status(&mut self, data_rdy: u16) -> Result<(), Icm20948Error> {
        let bytes = data_rdy.to_be_bytes();
        self.device.write_mems(dmp::DATA_RDY_STATUS, &bytes)
    }
    
    /// Set motion event control register
    pub fn set_motion_event_control(&mut self, motion_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = motion_mask.to_be_bytes();
        self.device.write_mems(dmp::MOTION_EVENT_CTL, &bytes)
    }
    
    /// Set sensor rate
    pub fn set_sensor_rate(&mut self, sensor: Sensor, divider: i16) -> Result<(), Icm20948Error> {
        let odr_addr = match sensor {
            Sensor::Accelerometer => dmp::ODR_ACCEL,
            Sensor::Gyroscope => dmp::ODR_GYRO,
            Sensor::Compass => dmp::ODR_CPASS,
            Sensor::ALS => dmp::ODR_ALS,
            Sensor::SixQ => dmp::ODR_QUAT6,
            Sensor::NineQ => dmp::ODR_QUAT9,
            Sensor::PedQ => dmp::ODR_PQUAT6,
            Sensor::GeoMag => dmp::ODR_GEOMAG,
            Sensor::Pressure => dmp::ODR_PRESSURE,
            Sensor::CalibGyro => dmp::ODR_GYRO_CALIBR,
            Sensor::CalibCompass => dmp::ODR_CPASS_CALIBR,
            _ => return Err(Icm20948Error::InvalidParameter),
        };

        // Write divider to the appropriate register
        let bytes = divider.to_be_bytes();
        self.device.write_mems(odr_addr, &bytes)
    }
    
    /// Set batch mode parameters
    pub fn set_batchmode_params(&mut self, thld: u32, mask: i16) -> Result<(), Icm20948Error> {
        // Reset batch counter
        let zeros = [0u8; 4];
        self.device.write_mems(dmp::BM_BATCH_CNTR, &zeros)?;
        
        // Set batch threshold
        let thld_bytes = thld.to_be_bytes();
        self.device.write_mems(dmp::BM_BATCH_THLD, &thld_bytes)?;
        
        // Set batch mask
        let mask_bytes = mask.to_be_bytes();
        self.device.write_mems(dmp::BM_BATCH_MASK, &mask_bytes)?;
        
        Ok(())
    }
    
    /// Set accelerometer bias
    pub fn set_bias_acc<'a>(&mut self, bias: &'a [i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::ACCEL_BIAS_X, &x_bytes)?;
        self.device.write_mems(dmp::ACCEL_BIAS_Y, &y_bytes)?;
        self.device.write_mems(dmp::ACCEL_BIAS_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Set gyroscope bias
    pub fn set_bias_gyr(&mut self, bias: &[i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::GYRO_BIAS_X, &x_bytes)?;
        self.device.write_mems(dmp::GYRO_BIAS_Y, &y_bytes)?;
        self.device.write_mems(dmp::GYRO_BIAS_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Set compass bias
    pub fn set_bias_cmp(&mut self, bias: &[i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::CPASS_BIAS_X, &x_bytes)?;
        self.device.write_mems(dmp::CPASS_BIAS_Y, &y_bytes)?;
        self.device.write_mems(dmp::CPASS_BIAS_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Get accelerometer bias
    pub fn get_bias_acc(&mut self) -> Result<[i32; 3], Icm20948Error> {
        let mut x_bytes = [0u8; 4];
        let mut y_bytes = [0u8; 4];
        let mut z_bytes = [0u8; 4];
        
        self.device.read_mems(dmp::ACCEL_BIAS_X, &mut x_bytes)?;
        self.device.read_mems(dmp::ACCEL_BIAS_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::ACCEL_BIAS_Z, &mut z_bytes)?;
        
        let x = i32::from_be_bytes(x_bytes);
        let y = i32::from_be_bytes(y_bytes);
        let z = i32::from_be_bytes(z_bytes);
        
        Ok([x, y, z])
    }
    
    /// Get gyroscope bias
    pub fn get_bias_gyr(&mut self) -> Result<[i32; 3], Icm20948Error> {
        let mut x_bytes = [0u8; 4];
        let mut y_bytes = [0u8; 4];
        let mut z_bytes = [0u8; 4];
        
        self.device.read_mems(dmp::GYRO_BIAS_X, &mut x_bytes)?;
        self.device.read_mems(dmp::GYRO_BIAS_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::GYRO_BIAS_Z, &mut z_bytes)?;
        
        let x = i32::from_be_bytes(x_bytes);
        let y = i32::from_be_bytes(y_bytes);
        let z = i32::from_be_bytes(z_bytes);
        
        Ok([x, y, z])
    }
    
    /// Get compass bias
    pub fn get_bias_cmp(&mut self) -> Result<[i32; 3], Icm20948Error> {
        let mut x_bytes = [0u8; 4];
        let mut y_bytes = [0u8; 4];
        let mut z_bytes = [0u8; 4];
        
        self.device.read_mems(dmp::CPASS_BIAS_X, &mut x_bytes)?;
        self.device.read_mems(dmp::CPASS_BIAS_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::CPASS_BIAS_Z, &mut z_bytes)?;
        
        let x = i32::from_be_bytes(x_bytes);
        let y = i32::from_be_bytes(y_bytes);
        let z = i32::from_be_bytes(z_bytes);
        
        Ok([x, y, z])
    }
    
    /// Set gyro scale factor
    pub fn set_gyro_sf(&mut self, gyro_sf: i32) -> Result<(), Icm20948Error> {
        let bytes = gyro_sf.to_be_bytes();
        self.device.write_mems(dmp::GYRO_SF, &bytes)
    }
    
    /// Set accel feedback gain
    pub fn set_accel_feedback_gain(&mut self, accel_gain: i32) -> Result<(), Icm20948Error> {
        let bytes = accel_gain.to_be_bytes();
        self.device.write_mems(dmp::ACCEL_ONLY_GAIN, &bytes)
    }
    
    /// Set accel calibration parameters
    pub fn set_accel_cal_params(&mut self, accel_cal: &[i32; 3]) -> Result<(), Icm20948Error> {
        let alpha_var_bytes = accel_cal[0].to_be_bytes();
        let a_var_bytes = accel_cal[1].to_be_bytes();
        let div_bytes = accel_cal[2].to_be_bytes();
        
        self.device.write_mems(dmp::ACCEL_ALPHA_VAR, &alpha_var_bytes)?;
        self.device.write_mems(dmp::ACCEL_A_VAR, &a_var_bytes)?;
        
        // Only write lower 16 bits of div
        let div_short_bytes = [(div_bytes[2]), (div_bytes[3])];
        self.device.write_mems(dmp::ACCEL_CAL_RATE, &div_short_bytes)?;
        
        Ok(())
    }
    
    /// Set compass cal parameters
    pub fn set_compass_cal_params(&mut self, compass_cal: &[i32; 2]) -> Result<(), Icm20948Error> {
        let time_buffer_bytes = (compass_cal[0] as i16).to_be_bytes();
        let radius_thresh_bytes = compass_cal[1].to_be_bytes();
        
        self.device.write_mems(dmp::CPASS_TIME_BUFFER, &time_buffer_bytes)?;
        self.device.write_mems(dmp::CPASS_RADIUS_3D_THRESH_ANOMALY, &radius_thresh_bytes)?;
        
        Ok(())
    }
    
    /// Set compass orientation matrix
    pub fn set_compass_matrix(&mut self, compass_mtx: &[i32; 9]) -> Result<(), Icm20948Error> {
        let mtx_addrs = [
            dmp::CPASS_MTX_00, dmp::CPASS_MTX_01, dmp::CPASS_MTX_02,
            dmp::CPASS_MTX_10, dmp::CPASS_MTX_11, dmp::CPASS_MTX_12,
            dmp::CPASS_MTX_20, dmp::CPASS_MTX_21, dmp::CPASS_MTX_22,
        ];
        
        for i in 0..9 {
            let bytes = compass_mtx[i].to_be_bytes();
            self.device.write_mems(mtx_addrs[i], &bytes)?;
        }
        
        Ok(())
    }
    
    /// Get pedometer step count
    pub fn get_pedometer_num_of_steps(&mut self) -> Result<u32, Icm20948Error> {
        let mut bytes = [0u8; 4];
        self.device.read_mems(dmp::PEDSTD_STEPCTR, &mut bytes)?;
        
        Ok(u32::from_be_bytes(bytes))
    }
    
    /// Set pedometer rate
    pub fn set_pedometer_rate(&mut self, _ped_rate: i32) -> Result<(), Icm20948Error> {
        // Not implemented in the C code either (commented out)
        Ok(())
    }
    
    /// Enable/disable wake on motion
    pub fn set_wom_enable(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let bytes = [0, if enable { 1 } else { 0 }];
        self.device.write_mems(dmp::WOM_ENABLE, &bytes)
    }
    
    /// Set wake on motion threshold
    pub fn set_wom_motion_threshold(&mut self, threshold: i32) -> Result<(), Icm20948Error> {
        let bytes = threshold.to_be_bytes();
        self.device.write_mems(dmp::WOM_THRESHOLD, &bytes)
    }
    
    /// Set wake on motion time threshold
    pub fn set_wom_time_threshold(&mut self, threshold: u16) -> Result<(), Icm20948Error> {
        let bytes = threshold.to_be_bytes();
        self.device.write_mems(dmp::WOM_CNTR_TH, &bytes)
    }
    
    /// Set gyro full-scale range
    pub fn set_gyro_fsr(&mut self, gyro_fsr_dps: u16) -> Result<(), Icm20948Error> {
        let scale = match gyro_fsr_dps {
            250 => 536870912i32, // 2^29
            500 => 268435456i32, // 2^28
            1000 => 134217728i32, // 2^27
            2000 => 67108864i32,   // 2^26
            4000 => 33554432i32,   // 2^25
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        let bytes = scale.to_be_bytes();
        self.device.write_mems(dmp::GYRO_FULLSCALE, &bytes)?;
        
        // Actualizar también la escala en el dispositivo principal si es necesario
        let gyro_scale = match gyro_fsr_dps {
            250 => GyroFullScale::Fs250Dps,
            500 => GyroFullScale::Fs500Dps,
            1000 => GyroFullScale::Fs1000Dps,
            2000 | _ => GyroFullScale::Fs2000Dps,
        };
        
        self.device.set_gyro_fullscale(gyro_scale)
    }
    
    /// Set accelerometer full-scale range
    pub fn set_accel_fsr(&mut self, accel_fsr_g: u16) -> Result<(), Icm20948Error> {
        let scale = match accel_fsr_g {
            2 => 33554432i32,  // 2^25
            4 => 67108864i32,  // 2^26
            8 => 134217728i32, // 2^27
            16 => 268435456i32, // 2^28
            32 => 536870912i32, // 2^29
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        let bytes = scale.to_be_bytes();
        self.device.write_mems(dmp::ACC_SCALE, &bytes)?;
        
        // Actualizar también la escala en el dispositivo principal si es necesario
        let accel_scale = match accel_fsr_g {
            2 => AccelFullScale::Fs2G,
            4 => AccelFullScale::Fs4G,
            8 => AccelFullScale::Fs8G,
            16 | _ => AccelFullScale::Fs16G,
        };
        
        self.device.set_accel_fullscale(accel_scale)
    }
    
    /// Set accelerometer scale2 factor
    pub fn set_accel_scale2(&mut self, accel_fsr: i16) -> Result<(), Icm20948Error> {
        let scale = match accel_fsr {
            2 => 524288i32,  // 2^19
            4 => 262144i32,  // 2^18
            8 => 131072i32,  // 2^17
            16 => 65536i32,  // 2^16
            32 => 32768i32,  // 2^15
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        let bytes = scale.to_be_bytes();
        self.device.write_mems(dmp::ACC_SCALE2, &bytes)
    }
    
    /// Set EIS authentication input
    pub fn set_eis_auth_input(&mut self, eis_auth_input: i32) -> Result<(), Icm20948Error> {
        let bytes = eis_auth_input.to_be_bytes();
        self.device.write_mems(dmp::EIS_AUTH_INPUT, &bytes)
    }
    
    /// Get EIS authentication output
    pub fn get_eis_auth_output(&mut self) -> Result<i32, Icm20948Error> {
        let mut bytes = [0u8; 4];
        self.device.read_mems(dmp::EIS_AUTH_OUTPUT, &mut bytes)?;
        
        Ok(i32::from_be_bytes(bytes))
    }
    
    /// Set BAC rate
    pub fn set_bac_rate(&mut self, bac_odr: DmpAlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match bac_odr {
            DmpAlgoFreq::Freq56Hz => 0i16,
            DmpAlgoFreq::Freq112Hz => 1i16,
            DmpAlgoFreq::Freq225Hz => 3i16,
            DmpAlgoFreq::Freq450Hz => 7i16,
            DmpAlgoFreq::Freq900Hz => 15i16,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::BAC_RATE, &bytes)
    }
    
    /// Set B2S rate
    pub fn set_b2s_rate(&mut self, accel_odr: DmpAlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match accel_odr {
            DmpAlgoFreq::Freq56Hz => 0i16,
            DmpAlgoFreq::Freq112Hz => 1i16,
            DmpAlgoFreq::Freq225Hz => 3i16,
            DmpAlgoFreq::Freq450Hz => 7i16,
            DmpAlgoFreq::Freq900Hz => 15i16,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::B2S_RATE, &bytes)
    }
    
    /// Set B2S orientation matrix
    pub fn set_b2s_matrix(&mut self, b2s_mtx: &[i32; 9]) -> Result<(), Icm20948Error> {
        let mtx_addrs = [
            dmp::B2S_MTX_00, dmp::B2S_MTX_01, dmp::B2S_MTX_02,
            dmp::B2S_MTX_10, dmp::B2S_MTX_11, dmp::B2S_MTX_12,
            dmp::B2S_MTX_20, dmp::B2S_MTX_21, dmp::B2S_MTX_22,
        ];
        
        for i in 0..9 {
            let bytes = b2s_mtx[i].to_be_bytes();
            self.device.write_mems(mtx_addrs[i], &bytes)?;
        }
        
        Ok(())
    }
    
    /// Set Flip-Pickup rate
    pub fn set_fp_rate(&mut self, accel_odr: DmpAlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match accel_odr {
            DmpAlgoFreq::Freq56Hz => 0i32,
            DmpAlgoFreq::Freq112Hz => 1i32,
            DmpAlgoFreq::Freq225Hz => 3i32,
            DmpAlgoFreq::Freq450Hz => 7i32,
            DmpAlgoFreq::Freq900Hz => 15i32,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::FP_RATE, &bytes)
    }
    
    /// Reset BAC states
    pub fn reset_bac_states(&mut self) -> Result<(), Icm20948Error> {
        let zero_i32 = [0u8; 4];
        let zero_i16 = [0u8; 2];
        
        // Reset all BAC state variables
        let i32_addrs = [
            dmp::BAC_STATE, dmp::BAC_STATE_PREV, 
            dmp::BAC_ACT_ON, dmp::BAC_ACT_OFF,
            dmp::BAC_STILL_S_F, dmp::BAC_RUN_S_F,
            dmp::BAC_DRIVE_S_F, dmp::BAC_WALK_S_F,
            dmp::BAC_SMD_S_F, dmp::BAC_BIKE_S_F,
            dmp::BAC_E1_SHORT, dmp::BAC_E2_SHORT,
            dmp::BAC_E3_SHORT, dmp::BAC_VAR_RUN,
            dmp::BAC_DRIVE_CONFIDENCE, dmp::BAC_WALK_CONFIDENCE,
            dmp::BAC_SMD_CONFIDENCE, dmp::BAC_BIKE_CONFIDENCE,
            dmp::BAC_STILL_CONFIDENCE, dmp::BAC_RUN_CONFIDENCE,
            dmp::BAC_MODE_CNTR, dmp::BAC_STATE_T_PREV,
            dmp::BAC_ACT_T_ON, dmp::BAC_ACT_T_OFF,
            dmp::BAC_STATE_WRDBS_PREV, dmp::BAC_ACT_WRDBS_ON,
            dmp::BAC_ACT_WRDBS_OFF
        ];
        
        for addr in i32_addrs.iter() {
            self.device.write_mems(*addr, &zero_i32)?;
        }
        
        let i16_addrs = [
            dmp::BAC_ACT_ON_OFF, dmp::PREV_BAC_ACT_ON_OFF,
            dmp::BAC_CNTR
        ];
        
        for addr in i16_addrs.iter() {
            self.device.write_mems(*addr, &zero_i16)?;
        }
        
        Ok(())
    }
    
    /// Set pedometer y-ratio
    pub fn set_ped_y_ratio(&mut self, ped_y_ratio: i32) -> Result<(), Icm20948Error> {
        let bytes = ped_y_ratio.to_be_bytes();
        self.device.write_mems(dmp::PED_Y_RATIO, &bytes)
    }
    
    /// Set orientation parameters
    pub fn set_orientation_params(&mut self, orientation_params: &[i32; 4]) -> Result<(), Icm20948Error> {
        let q0_bytes = orientation_params[0].to_be_bytes();
        let q1_bytes = orientation_params[1].to_be_bytes();
        let q2_bytes = orientation_params[2].to_be_bytes();
        let q3_bytes = orientation_params[3].to_be_bytes();
        
        self.device.write_mems(dmp::Q0_QUAT6, &q0_bytes)?;
        self.device.write_mems(dmp::Q1_QUAT6, &q1_bytes)?;
        self.device.write_mems(dmp::Q2_QUAT6, &q2_bytes)?;
        self.device.write_mems(dmp::Q3_QUAT6, &q3_bytes)?;
        
        Ok(())
    }
}
