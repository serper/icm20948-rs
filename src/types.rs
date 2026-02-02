//! Definiciones de tipos y constantes comunes para el ICM20948

/// Maximum size for serial read operations
pub const INV_MAX_SERIAL_READ: usize = 16;
/// Maximum size for serial write operations
pub const INV_MAX_SERIAL_WRITE: usize = 16;
/// DMP firmware load address
pub const DMP_LOAD_START: u16 = 0x90;
/// DMP firmware start address
pub const DMP_START_ADDRESS: u16 = 0x1000;

/// Escalas completas disponibles para el giroscopio
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GyroFullScale {
    /// ±250 dps
    Fs250Dps = 0,
    /// ±500 dps
    Fs500Dps = 1,
    /// ±1000 dps
    Fs1000Dps = 2,
    /// ±2000 dps
    #[default]
    Fs2000Dps = 3,
}

// Añadimos implementación para convertir desde u8
impl From<u8> for GyroFullScale {
    fn from(value: u8) -> Self {
        match value & 0x03 {
            0 => GyroFullScale::Fs250Dps,
            1 => GyroFullScale::Fs500Dps,
            2 => GyroFullScale::Fs1000Dps,
            _ => GyroFullScale::Fs2000Dps,
        }
    }
}

/// Escalas completas disponibles para el acelerómetro
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum AccelFullScale {
    /// ±2g
    #[default]
    Fs2G = 0,
    /// ±4g
    Fs4G = 1,
    /// ±8g
    Fs8G = 2,
    /// ±16g
    Fs16G = 3,
}

// Añadimos implementación para convertir desde u8
impl From<u8> for AccelFullScale {
    fn from(value: u8) -> Self {
        match value & 0x03 {
            0 => AccelFullScale::Fs2G,
            1 => AccelFullScale::Fs4G,
            2 => AccelFullScale::Fs8G,
            _ => AccelFullScale::Fs16G,
        }
    }
}

/// Bits útiles para configuración y control
pub mod bits {
    // Power management bits
    pub const H_RESET: u8 = 0x80;
    pub const SLEEP: u8 = 0x40;
    pub const LP_EN: u8 = 0x20;
    pub const CHIP_AWAKE: u8 = 0x40;

    // Mode bits para LP_CONFIG
    pub const GYRO_CYCLE: u8 = 0x10;
    pub const ACCEL_CYCLE: u8 = 0x20;
    pub const I2C_MST_CYCLE: u8 = 0x40;

    // Bits de máscara para selección de escalas
    pub const GYRO_FS_SEL: u8 = 0x06; // Bits [2:1]
    pub const ACCEL_FS_SEL: u8 = 0x06; // Bits [2:1]

    // Bits de interrupción
    pub const INT1_ACTL: u8 = 0x80;
    pub const INT_ANYRD_2CLEAR: u8 = 0x10;
    pub const INT_BYPASS_EN: u8 = 0x02;
    pub const INT_RAW_DATA_RDY_EN: u8 = 0x01;

    // Otros bits de control
    pub const FIFO_EN: u8 = 0x40;
    pub const DMP_EN: u8 = 0x80;
    pub const FIFO_RST: u8 = 0x04;
    pub const CLK_PLL: u8 = 0x01;
    pub const PWR_ACCEL_STBY: u8 = 0x38; // bits 5:3
    pub const PWR_GYRO_STBY: u8 = 0x07; // bits 2:0
    pub const PWR_PRESSURE_STBY: u8 = 0x80;
    pub const I2C_IF_DIS: u8 = 0x10;

    pub const DMP_RST: u8 = 0x08;
    pub const I2C_MST_EN: u8 = 0x20;
    pub const I2C_BYPASS_EN: u8 = 0x02;
    pub const SINGLE_FIFO_CFG: u8 = 0x01;
    pub const DMP_INT_EN: u8 = 0x02;
    pub const I2C_MST_P_NSR: u8 = 0x10;
    pub const I2C_MST_RST: u8 = 0x02;

    pub const DATA_RDY_3_EN: u8 = 0x08;
    pub const DATA_RDY_2_EN: u8 = 0x04;
    pub const DATA_RDY_1_EN: u8 = 0x02;
    pub const DATA_RDY_0_EN: u8 = 0x01;

    pub const WAKE_ON_MOTION_INT: u8 = 0x08; // Updated to use const
    pub const MSG_DMP_INT: u16 = 0x0002;
    pub const MSG_DMP_INT_0: u16 = 0x0100; // CI Command

    pub const MSG_DMP_INT_2: u16 = 0x0200; // CIM Command - SMD
    pub const MSG_DMP_INT_3: u16 = 0x0400; // CIM Command - Pedometer

    pub const MSG_DMP_INT_4: u16 = 0x1000; // CIM Command - Pedometer binning
    pub const MSG_DMP_INT_5: u16 = 0x2000; // CIM Command - Bring To See Gesture
    pub const MSG_DMP_INT_6: u16 = 0x4000; // CIM Command - Look To See Gesture
    pub const LPF_SETTING: u8 = 0b00111001; // bits[5:3] = DLFCFG, bits[2:1] = FSCALE, bits[0] = FCHOICE
}

/// Valores de aceleración gravitacional en diferentes unidades
pub mod gravity {
    pub const GRAVITY_MSS: f32 = 9.80665;
    pub const GRAVITY_FPS2: f32 = 32.17405;
}

/// Factores de conversión para los diferentes modelos de magnetómetros (en micro Teslas * 2^30)
pub mod scale_factor {
    pub const AK8975: i32 = 322122547; // 0.3 µT * (1 << 30)
    pub const AK8972: i32 = 644245094; // 0.6 µT * (1 << 30)
    pub const AK8963_14BIT: i32 = 644245094; // 0.6 µT * (1 << 30)
    pub const AK8963_16BIT: i32 = 161061273; // 0.15 µT * (1 << 30)
    pub const AK09911: i32 = 644245094; // 0.6 µT * (1 << 30)
    pub const AK09912: i32 = 161061273; // 0.15 µT * (1 << 30)
    pub const AK09916: i32 = 161061273; // 0.15 µT * (1 << 30)
}

/// Valores específicos para registros del magnetómetro
pub mod ak_val {
    pub const WIA_VAL: u8 = 0x48; // Valor esperado en registro WIA
    pub const POWER_DOWN: u8 = 0x00; // Modo power down
    pub const SINGLE_MEASURE: u8 = 0x01; // Modo de medición única
    pub const FUSE_ROM_ACCESS: u8 = 0x0F; // Modo de acceso a ROM de calibración
    pub const AK09911_FUSE_ROM: u8 = 0x1F; // Modo ROM para AK09911
    pub const AK09912_FUSE_ROM: u8 = 0x1F; // Modo ROM para AK09912
    pub const AK09916_MODE_ST: u8 = 0x10; // Modo self-test para AK09916
    pub const SELF_TEST: u8 = 0x40; // Bit de self-test
    pub const DRDY: u8 = 0x01; // Bit de datos listos
    pub const DOR: u8 = 0x02; // Bit de overrun
}

/// Bits para configuración de I2C secundario
pub mod slv_bits {
    pub const I2C_READ: u8 = 0x80; // Bit de lectura para dirección I2C
    pub const I2C_ENABLE: u8 = 0x80; // Bit de habilitación para esclavo
    pub const I2C_BYTE_SWAP: u8 = 0x40; // Bit de swap de bytes
    pub const I2C_REG_DIS: u8 = 0x20; // Bit para deshabilitar registro
    pub const I2C_GRP: u8 = 0x10; // Bit para agrupar
    pub const I2C_BYTE_LEN_MASK: u8 = 0x0F; // Máscara para longitud de bytes
}

/// Data Definitions
pub mod data_defs {
    pub const BYTES_PER_SENSOR: u8 = 6;
    pub const FIFO_COUNT_BYTE: u8 = 2;
    pub const HARDWARE_FIFO_SIZE: u16 = 1024;

    pub const FIFO_SIZE: u16 = HARDWARE_FIFO_SIZE * 7 / 8;
    pub const POWER_UP_TIME: u16 = 100;
    pub const REG_UP_TIME_USEC: u16 = 100;
    pub const DMP_RESET_TIME: u16 = 20;

    pub const GYRO_ENGINE_UP_TIME: u16 = 50;
    pub const MPU_MEM_BANK_SIZE: u16 = 256;
    pub const IIO_BUFFER_BYTES: u8 = 8;
    pub const HEADERED_NORMAL_BYTES: u8 = 8;
    pub const HEADERED_Q_BYTES: u8 = 16;
    pub const LEFT_OVER_BYTES: u16 = 128;
    pub const BASE_SAMPLE_RATE: u16 = 1125;

    pub const MPU_DEFAULT_DMP_FREQ: u16 = 102;
    pub const PEDOMETER_FREQ: u16 = MPU_DEFAULT_DMP_FREQ >> 1;
    pub const DEFAULT_ACCEL_GAIN: i64 = 33554432;
    pub const PED_ACCEL_GAIN: i64 = 67108864;
    pub const ALPHA_FILL_PED: i64 = 858993459;
    pub const A_FILL_PED: i64 = 214748365;

    pub const MIN_MST_ODR_CONFIG: u8 = 4;
    pub const THREE_AXES: u8 = 3;
    pub const NINE_ELEM: u8 = THREE_AXES * THREE_AXES;
    pub const MPU_TEMP_SHIFT: u8 = 16;
    pub const SOFT_IRON_MATRIX_SIZE: u8 = 4 * 9;
    pub const DMP_DIVIDER: u16 = BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ;
    pub const MAX_5_BIT_VALUE: u8 = 0x1F;
    pub const BAD_COMPASS_DATA: u16 = 0x7FFF;
    pub const DEFAULT_BATCH_RATE: u16 = 400;
    pub const DEFAULT_BATCH_TIME: u16 = 1000 / DEFAULT_BATCH_RATE;
    pub const MAX_COMPASS_RATE: u8 = 115;
    pub const MAX_PRESSURE_RATE: u8 = 30;
    pub const MAX_ALS_RATE: u8 = 5;
    pub const DATA_AKM_99_BYTES_DMP: u8 = 10;
    pub const DATA_AKM_89_BYTES_DMP: u8 = 9;
    pub const DATA_ALS_BYTES_DMP: u8 = 8;
    pub const APDS9900_AILTL_REG: u8 = 0x04;
    pub const BMP280_DIG_T1_LSB_REG: u8 = 0x88;
    pub const COVARIANCE_SIZE: u8 = 14;
    pub const ACCEL_COVARIANCE_SIZE: usize = COVARIANCE_SIZE as usize * std::mem::size_of::<i16>();
    pub const COMPASS_COVARIANCE_SIZE: usize =
        COVARIANCE_SIZE as usize * std::mem::size_of::<i16>();
    pub const TEMPERATURE_SCALE: i64 = 3340827;
    pub const TEMPERATURE_OFFSET: i64 = 1376256;
    pub const SECONDARY_INIT_WAIT: u8 = 60;
    pub const MPU_SOFT_UPDT_ADDR: u8 = 0x86;
    pub const MPU_SOFT_UPTD_MASK: u8 = 0x0F;
    pub const AK99XX_SHIFT: u8 = 23;
    pub const AK89XX_SHIFT: u8 = 22;
    pub const OPERATE_GYRO_IN_DUTY_CYCLED_MODE: u8 = 1 << 4;
    pub const OPERATE_ACCEL_IN_DUTY_CYCLED_MODE: u8 = 1 << 5;
    pub const OPERATE_I2C_MASTER_IN_DUTY_CYCLED_MODE: u8 = 1 << 6;
}

// Constantes para los encabezados del DMP
#[allow(non_upper_case_globals)]
pub mod dmp_header {
    pub const PED_STEPIND: u16 = 0x0007; // Number of steps detected in 3 LSBs of header
    pub const HEADER2: u16 = 0x0008; // Enable/disable data output in data output control register 2
    pub const STEP_DETECTOR: u16 = 0x0010; // Timestamp when each step is detected
    pub const COMPASS_CALIBR: u16 = 0x0020; // Calibrated magnetic
    pub const GYRO_CALIBR: u16 = 0x0040; // Calibrated gyro
    pub const PRESSURE: u16 = 0x0080; // Pressure
    pub const GEOMAG: u16 = 0x0100; // Geomagnetic rotation vector with heading accuracy
    pub const PQUAT6: u16 = 0x0200; // Truncated game rotation vector for batching
    pub const QUAT9: u16 = 0x0400; // Rotation vector with heading accuracy
    pub const QUAT6: u16 = 0x0800; // Game rotation vector
    pub const ALS: u16 = 0x1000; // Ambient light sensor
    pub const COMPASS: u16 = 0x2000; // Compass
    pub const GYRO: u16 = 0x4000; // Gyroscope
    pub const ACCEL: u16 = 0x8000; // Accelerometer
    pub const EMPTY: u16 = 0xFFFF; // Empty value
}

#[allow(non_upper_case_globals)]
pub mod dmp_header2 {
    pub const SECONDARY_ON_OFF: u16 = 0x0040; // Enable/disable secondary sensor
    pub const ACTIVITY_RECOG: u16 = 0x0080; // Activity recognition engine
    pub const BATCH_MODE_EN: u16 = 0x0100; // Enable batching (HEADER2)
    pub const PICKUP: u16 = 0x0400; // Flip/pick-up gesture detector (HEADER2)
    pub const FSYNC: u16 = 0x0800; // Frame sync from camera sensor (HEADER2)
    pub const COMPASS_ACCURACY: u16 = 0x1000; // Compass accuracy when changes (HEADER2)
    pub const GYRO_ACCURACY: u16 = 0x2000; // Gyro accuracy when changes (HEADER2)
    pub const ACCEL_ACCURACY: u16 = 0x4000; // Accel accuracy when changes (HEADER2)
}

#[allow(non_upper_case_globals)]
pub mod dmp_packet_bytes {
    pub const HEADER: usize = 2;
    pub const HEADER2: usize = 2;
    pub const RAW_ACCEL: usize = 6;
    pub const RAW_GYRO: usize = 6;
    pub const GYRO_BIAS: usize = 6;
    pub const COMPASS: usize = 6;
    pub const ALS: usize = 8;
    pub const QUAT6: usize = 12;
    pub const QUAT9: usize = 14;
    pub const PQUAT6: usize = 6;
    pub const GEOMAG: usize = 14;
    pub const PRESSURE: usize = 6;
    pub const GYRO_CALIBR: usize = 12;
    pub const COMPASS_CALIBR: usize = 12;
    pub const STEP_DETECTOR: usize = 4;
    pub const STEP_COUNTER: usize = 2;
    pub const FOOTER: usize = 2;
    pub const MAXIMUM: usize = 14;
}

#[allow(non_upper_case_globals)]
pub mod dmp_packet_bytes2 {
    pub const ACCEL_ACCURACY: usize = 2;
    pub const GYRO_ACCURACY: usize = 2;
    pub const COMPASS_ACCURACY: usize = 2;
    pub const FSYNC: usize = 2;
    pub const PICKUP: usize = 2;
    pub const BATCH_MODE: usize = 2;
    pub const ACTIVITY_RECOG: usize = 6;
    pub const SECONDARY_ON_OFF: usize = 2;
}

// pub enum Sensor {
//     Accel,
//     Gyro,
//     Cpass,
//     Als,
//     Quat6,
//     Quat9,
//     Pquat6,
//     Geomag,
//     Pressure,
//     GyroCalibr,
//     CpassCalibr,
//     StepDetector,
//     Header2,
//     PedometerStepIndicatorBits,
// }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Sensor {
    Accelerometer,
    Gyroscope,
    RawAccelerometer,
    RawGyroscope,
    MagneticFieldUncalibrated,
    GyroscopeUncalibrated,
    ActivityClassification,
    StepDetector,
    StepCounter,
    GameRotationVector,
    RotationVector,
    GeomagneticRotationVector,
    GeomagneticField,
    WakeupSignificantMotion,
    FlipPickup,
    WakeupTiltDetector,
    Gravity,
    LinearAcceleration,
    Orientation,
    B2S,
    RawMagnetometer,
    Max,
}

#[derive(Clone, Copy)]
pub enum AndroidSensor {
    MetaData,                        // 0
    Accelerometer,                   // 1
    GeomagneticField,                // 2
    Orientation,                     // 3
    Gyroscope,                       // 4
    Light,                           // 5
    Pressure,                        // 6
    Temperature,                     // 7
    WakeupProximity,                 // 8
    Gravity,                         // 9
    LinearAcceleration,              // 10
    RotationVector,                  // 11
    Humidity,                        // 12
    AmbientTemperature,              // 13
    MagneticFieldUncalibrated,       // 14
    GameRotationVector,              // 15
    GyroscopeUncalibrated,           // 16
    WakeupSignificantMotion,         // 17
    StepDetector,                    // 18
    StepCounter,                     // 19
    GeomagneticRotationVector,       // 20
    HeartRate,                       // 21
    Proximity,                       // 22
    WakeupAccelerometer,             // 23
    WakeupMagneticField,             // 24
    WakeupOrientation,               // 25
    WakeupGyroscope,                 // 26
    WakeupLight,                     // 27
    WakeupPressure,                  // 28
    WakeupGravity,                   // 29
    WakeupLinearAcceleration,        // 30
    WakeupRotationVector,            // 31
    WakeupRelativeHumidity,          // 32
    WakeupAmbientTemperature,        // 33
    WakeupMagneticFieldUncalibrated, // 34
    WakeupGameRotationVector,        // 35
    WakeupGyroscopeUncalibrated,     // 36
    WakeupStepDetector,              // 37
    WakeupStepCounter,               // 38
    WakeupGeomagneticRotationVector, // 39
    WakeupHeartRate,                 // 40
    WakeupTiltDetector,              // 41
    RawAccelerometer,                // 42
    RawGyroscope,                    // 43
    NumMax,                          // 44
    B2S,                             // 45
    FlipPickup,                      // 46
    ActivityClassification,          // 47
    ScreenRotation,                  // 48
    SelfTest,                        // 49
    Setup,                           // 50
    GeneralSensorsMax,               // 51
}

pub const SENSORTOCONTROLBITS: &[u16] = &[
    // Data output control 1 register bit definition
    // 16-bit accel                                0x8000
    // 16-bit gyro                                 0x4000
    // 16-bit compass                              0x2000
    // 16-bit ALS                                  0x1000
    // 32-bit 6-axis quaternion                    0x0800
    // 32-bit 9-axis quaternion + heading accuracy 0x0400
    // 16-bit pedometer quaternion                 0x0200
    // 32-bit Geomag rv + heading accuracy         0x0100
    // 16-bit Pressure                             0x0080
    // 32-bit calibrated gyro                      0x0040
    // 32-bit calibrated compass                   0x0020
    // Pedometer Step Detector                     0x0010
    // Header 2                                    0x0008
    // Pedometer Step Indicator Bit 2              0x0004
    // Pedometer Step Indicator Bit 1              0x0002
    // Pedometer Step Indicator Bit 0              0x0001
    // Unsupported Sensors are 0xFFFF
    0xFFFF, // 0  Meta Data
    0x8008, // 1  Accelerometer
    0x0028, // 2  Magnetic Field
    0x0408, // 3  Orientation
    0x4048, // 4  Gyroscope
    0x1008, // 5  Light
    0x0088, // 6  Pressure
    0xFFFF, // 7  Temperature
    0xFFFF, // 8  Proximity <----------- fixme
    0x0808, // 9  Gravity
    0x8808, // 10 Linear Acceleration
    0x0408, // 11 Rotation Vector
    0xFFFF, // 12 Humidity
    0xFFFF, // 13 Ambient Temperature
    0x2008, // 14 Magnetic Field Uncalibrated
    0x0808, // 15 Game Rotation Vector
    0x4008, // 16 Gyroscope Uncalibrated
    0x0000, // 17 Significant Motion
    0x0018, // 18 Step Detector
    0x0010, // 19 Step Counter <----------- fixme
    0x0108, // 20 Geomagnetic Rotation Vector
    0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
    0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,
    0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
    0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
    0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
    0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
    0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
    0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
    0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
    0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
    0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
    0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
    0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
    0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
    0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
    0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
    0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
    0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
    0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
    0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
    0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
    0x8008, // 42 Raw Acc
    0x4048, // 43 Raw Gyr
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OdrSensor {
    Accel,
    Gyro,
    Cpass,
    Als,
    Quat6,
    Quat9,
    Pquat6,
    Geomag,
    Pressure,
    GyroCalibr,
    CpassCalibr,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Activity {
    Drive = 0x01,
    Walk = 0x02,
    Run = 0x04,
    Bike = 0x08,
    Tilt = 0x10,
    Still = 0x20,
}
