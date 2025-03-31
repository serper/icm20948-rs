//! Definiciones de tipos y constantes comunes para el ICM20948

/// Escalas completas disponibles para el giroscopio
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroFullScale {
    /// ±250 dps
    Fs250Dps = 0,
    /// ±500 dps
    Fs500Dps = 1,
    /// ±1000 dps
    Fs1000Dps = 2,
    /// ±2000 dps
    Fs2000Dps = 3,
}

impl Default for GyroFullScale {
    fn default() -> Self {
        GyroFullScale::Fs2000Dps
    }
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelFullScale {
    /// ±2g
    Fs2G = 0,
    /// ±4g
    Fs4G = 1,
    /// ±8g
    Fs8G = 2,
    /// ±16g
    Fs16G = 3,
}

impl Default for AccelFullScale {
    fn default() -> Self {
        AccelFullScale::Fs2G
    }
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
    pub const GYRO_CYCLE: u8 = 0x80;
    pub const ACCEL_CYCLE: u8 = 0x40;

    // Bits de máscara para selección de escalas
    pub const GYRO_FS_SEL: u8 = 0x06; // Bits [2:1]
    pub const ACCEL_FS_SEL: u8 = 0x06; // Bits [2:1]
    
    // Bits de interrupción
    pub const INT1_ACTL: u8 = 0x80;
    pub const INT_ANYRD_2CLEAR: u8 = 0x10;
    pub const INT_BYPASS_EN: u8 = 0x02;
    
    // Otros bits de control
    pub const FIFO_EN: u8 = 0x40;
    pub const DMP_EN: u8 = 0x80;
    pub const FIFO_RST: u8 = 0x04;
    pub const CLK_PLL: u8 = 0x01;
    pub const PWR_ACCEL_STBY: u8 = 0x38;  // bits 5:3
    pub const PWR_GYRO_STBY: u8 = 0x07;   // bits 2:0
    pub const PWR_PRESSURE_STBY: u8 = 0x80;
    pub const I2C_IF_DIS: u8 = 0x10;
    
    pub const DMP_RST: u8 = 0x08;
    pub const I2C_MST_EN: u8 = 0x20;
    pub const I2C_MST_CYCLE: u8 = 0x40;
    pub const SINGLE_FIFO_CFG: u8 = 0x01;
    pub const DMP_INT_EN: u8 = 0x02;
    pub const I2C_MST_P_NSR: u8 = 0x10;

    pub const DATA_RDY_3_EN: u8 = 0x08;
    pub const DATA_RDY_2_EN: u8 = 0x04;
    pub const DATA_RDY_1_EN: u8 = 0x02;
    pub const DATA_RDY_0_EN: u8 = 0x01;

    pub const WAKE_ON_MOTION_INT: u8 = 0x08; // Updated to use const
    pub const MSG_DMP_INT: u16 = 0x0002;
    pub const MSG_DMP_INT_0: u16 = 0x0100;  // CI Command

    pub const MSG_DMP_INT_2: u16 = 0x0200;  // CIM Command - SMD
    pub const MSG_DMP_INT_3: u16 = 0x0400;  // CIM Command - Pedometer

    pub const MSG_DMP_INT_4: u16 = 0x1000;  // CIM Command - Pedometer binning
    pub const MSG_DMP_INT_5: u16 = 0x2000;  // CIM Command - Bring To See Gesture
    pub const MSG_DMP_INT_6: u16 = 0x4000;  // CIM Command - Look To See Gesture
    pub const LPF_SETTING: u8 = 0b00111001; // bits[5:3] = DLFCFG, bits[2:1] = FSCALE, bits[0] = FCHOICE
}

/// Valores de aceleración gravitacional en diferentes unidades
pub mod gravity {
    pub const GRAVITY_MSS: f32 = 9.80665;
    pub const GRAVITY_FPS2: f32 = 32.17405;
}

/// Factores de conversión para los diferentes modelos de magnetómetros (en micro Teslas * 2^30)
pub mod scale_factor {
    pub const AK8975: i32 = 322122547;  // 0.3 µT * (1 << 30)
    pub const AK8972: i32 = 644245094;  // 0.6 µT * (1 << 30)
    pub const AK8963_14BIT: i32 = 644245094;  // 0.6 µT * (1 << 30)
    pub const AK8963_16BIT: i32 = 161061273;  // 0.15 µT * (1 << 30)
    pub const AK09911: i32 = 644245094;  // 0.6 µT * (1 << 30)
    pub const AK09912: i32 = 161061273;  // 0.15 µT * (1 << 30)
    pub const AK09916: i32 = 161061273;  // 0.15 µT * (1 << 30)
}

/// Valores específicos para registros del magnetómetro
pub mod ak_val {
    pub const WIA_VAL: u8 = 0x48;     // Valor esperado en registro WIA
    pub const POWER_DOWN: u8 = 0x00;  // Modo power down
    pub const SINGLE_MEASURE: u8 = 0x01; // Modo de medición única
    pub const FUSE_ROM_ACCESS: u8 = 0x0F; // Modo de acceso a ROM de calibración
    pub const AK09911_FUSE_ROM: u8 = 0x1F; // Modo ROM para AK09911
    pub const AK09912_FUSE_ROM: u8 = 0x1F; // Modo ROM para AK09912
    pub const AK09916_MODE_ST: u8 = 0x10; // Modo self-test para AK09916
    pub const SELF_TEST: u8 = 0x40;   // Bit de self-test
    pub const DRDY: u8 = 0x01;        // Bit de datos listos
    pub const DOR: u8 = 0x02;         // Bit de overrun
}

/// Bits para configuración de I2C secundario
pub mod slv_bits {
    pub const I2C_READ: u8 = 0x80;  // Bit de lectura para dirección I2C
    pub const I2C_ENABLE: u8 = 0x80; // Bit de habilitación para esclavo
    pub const I2C_BYTE_SWAP: u8 = 0x40; // Bit de swap de bytes
    pub const I2C_REG_DIS: u8 = 0x20; // Bit para deshabilitar registro
    pub const I2C_GRP: u8 = 0x10;   // Bit para agrupar
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
    pub const COMPASS_COVARIANCE_SIZE: usize = COVARIANCE_SIZE as usize * std::mem::size_of::<i16>();
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
