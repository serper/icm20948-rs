//! Definiciones de registros para el ICM20948
//!
//! Se han actualizado los valores para que sean direcciones absolutas.

/// Definir el trait para los bancos

/// Definición de registros para cada Banco 
/// Registros del Banco 0
pub mod registers {
    pub trait RegisterBank {
        const BANK: u8;
    }

    pub mod bank0 {
        use super::RegisterBank;

        /// Registros del Banco 0
        pub struct Bank;
        impl RegisterBank for Bank {
            const BANK: u8 = 0;
        }

        // Registros de identificación
        pub const WHO_AM_I: u8 = 0x00;
        pub const USER_CTRL: u8 = 0x03;
        
        // Registros de estado y control
        pub const LP_CONFIG: u8 = 0x05;
        pub const PWR_MGMT_1: u8 = 0x06;
        pub const PWR_MGMT_2: u8 = 0x07;
        pub const INT_PIN_CFG: u8 = 0x0F;
        pub const INT_ENABLE: u8 = 0x10;
        pub const INT_ENABLE_1: u8 = 0x11;
        pub const INT_ENABLE_2: u8 = 0x12;
        pub const INT_ENABLE_3: u8 = 0x13;
        pub const I2C_MST_STATUS: u8 = 0x17;
        pub const DMP_INT_STATUS: u8 = 0x18;
        pub const INT_STATUS: u8 = 0x19;
        pub const INT_STATUS_1: u8 = 0x1A;
        pub const INT_STATUS_2: u8 = 0x1B;
        
        // Registros de acelerómetro
        pub const ACCEL_XOUT_H: u8 = 0x2D;
        pub const ACCEL_XOUT_L: u8 = 0x2E;
        pub const ACCEL_YOUT_H: u8 = 0x2F;
        pub const ACCEL_YOUT_L: u8 = 0x30;
        pub const ACCEL_ZOUT_H: u8 = 0x31;
        pub const ACCEL_ZOUT_L: u8 = 0x32;

        // Registros de giroscopio
        pub const GYRO_XOUT_H: u8 = 0x33;
        pub const GYRO_XOUT_L: u8 = 0x34;
        pub const GYRO_ZOUT_H: u8 = 0x35;
        pub const GYRO_ZOUT_L: u8 = 0x36;
        pub const GYRO_YOUT_H: u8 = 0x37;
        pub const GYRO_YOUT_L: u8 = 0x38;    
        
        // Registros de temperatura
        pub const TEMP_OUT_H: u8 = 0x39;
        pub const TEMP_OUT_L: u8 = 0x3A;
        
        // Registros de FIFO
        pub const FIFO_EN_1: u8 = 0x66;
        pub const FIFO_EN_2: u8 = 0x67;
        pub const FIFO_RST: u8 = 0x68;
        pub const FIFO_MODE: u8 = 0x69;
        pub const FIFO_COUNTH: u8 = 0x70;
        pub const FIFO_COUNTL: u8 = 0x71;
        pub const FIFO_R_W: u8 = 0x72;
        pub const HW_FIX_DISABLE: u8 = 0x75;
        pub const FIFO_CFG: u8 = 0x76;
        
        // Registros de MEM
        pub const MEM_START_ADDR: u8 = 0x7C;
        pub const MEM_R_W: u8 = 0x7D;
        pub const MEM_BANK_SEL: u8 = 0x7E;
        
        // Selección de banco de registros
        pub const REG_BANK_SEL: u8 = 0x7F;
    }

    /// Registros del Banco 1 (direcciones absolutas)
    pub mod bank1 {
        use super::RegisterBank;

        /// Registros del Banco 1
        pub struct Bank;
        impl RegisterBank for Bank {
            const BANK: u8 = 1;
        }

        // Registros de muestreo del sensor
        pub const SELF_TEST_X_GYRO: u8 = 0x02;
        pub const SELF_TEST_Y_GYRO: u8 = 0x03;
        pub const SELF_TEST_Z_GYRO: u8 = 0x04;
        pub const SELF_TEST_X_ACCEL: u8 = 0x0E;
        pub const SELF_TEST_Y_ACCEL: u8 = 0x0F;
        pub const SELF_TEST_Z_ACCEL: u8 = 0x10;

        // Registros de configuración de acelerómetro
        pub const XA_OFFS_H: u8 = 0x14;
        pub const XA_OFFS_L: u8 = 0x15;
        pub const YA_OFFS_H: u8 = 0x17;
        pub const YA_OFFS_L: u8 = 0x18;
        pub const ZA_OFFS_H: u8 = 0x1A;
        pub const ZA_OFFS_L: u8 = 0x1B;

        // Registros de corrección de tiempo
        pub const TIMEBASE_CORRECTION_PLL: u8 = 0x28;
        pub const TIMEBASE_CORRECTION_RCOSC: u8 = 0x29;
        
        // Selección de banco de registros
        pub const REG_BANK_SEL: u8 = 0x7F;
    }

    /// Registros del Banco 2 (direcciones absolutas)
    pub mod bank2 {
        use super::RegisterBank;
        /// Registros del Banco 2
        pub struct Bank;
        impl RegisterBank for Bank {
            const BANK: u8 = 2;
        }

        // Registros de configuración del giroscopio
        pub const GYRO_SMPLRT_DIV: u8 = 0x00;
        pub const GYRO_CONFIG_1: u8 = 0x01;
        pub const GYRO_CONFIG_2: u8 = 0x02;

        // Registros de compensación de giroscopio
        pub const XG_OFFS_USRH: u8 = 0x03;
        pub const XG_OFFS_USRL: u8 = 0x04;
        pub const YG_OFFS_USRH: u8 = 0x05;
        pub const YG_OFFS_USRL: u8 = 0x06;    
        pub const ZG_OFFS_USRH: u8 = 0x07;
        pub const ZG_OFFS_USRL: u8 = 0x08;

        // Registros de configuración del acelerómetro
        pub const ACCEL_SMPLRT_DIV_1: u8 = 0x10;
        pub const ACCEL_SMPLRT_DIV_2: u8 = 0x11;
        pub const ACCEL_CONFIG_1: u8 = 0x14;
        pub const ACCEL_CONFIG_2: u8 = 0x15;
        
        pub const ACCEL_INTEL_CTRL: u8 = 0x18;
        pub const ACCEL_WOM_THR: u8 = 0x19;
        pub const PRS_ODR_CONFIG: u8 = 0x20;
        pub const PRGM_START_ADDRH: u8 = 0x50;
        pub const MOD_CTRL_USR: u8 = 0x54;

        // Registros de detección de movimiento
        pub const WAKE_ON_MOTION_THRESHOLD: u8 = 0x13;
        
        // Selección de banco de registros
        pub const REG_BANK_SEL: u8 = 0x7F;
    }

    /// Registros del Banco 3 (direcciones absolutas)
    pub mod bank3 {
        use super::RegisterBank;
        /// Registros del Banco 3
        pub struct Bank;
        impl RegisterBank for Bank {
            const BANK: u8 = 3;
        }

        // Registros de configuración I2C
        pub const I2C_MST_ODR_CONFIG: u8 = 0x00;
        pub const I2C_MST_CTRL: u8 = 0x01;
        pub const I2C_MST_DELAY_CTRL: u8 = 0x02;
        pub const I2C_SLV0_ADDR: u8 = 0x03;
        pub const I2C_SLV0_REG: u8 = 0x04;
        pub const I2C_SLV0_CTRL: u8 = 0x05;
        pub const I2C_SLV0_DO: u8 = 0x06;
        
        // Registros para comunicación con magnetómetro
        pub const I2C_SLV1_ADDR: u8 = 0x07;
        pub const I2C_SLV1_REG: u8 = 0x08;
        pub const I2C_SLV1_CTRL: u8 = 0x09;
        pub const I2C_SLV1_DO: u8 = 0x0A;
        
        // Registros para dispositivos adicionales
        pub const I2C_SLV2_ADDR: u8 = 0x0B;
        pub const I2C_SLV2_REG: u8 = 0x0C;
        pub const I2C_SLV2_CTRL: u8 = 0x0D;
        pub const I2C_SLV2_DO: u8 = 0x0E;
        pub const I2C_SLV3_ADDR: u8 = 0x0F;
        pub const I2C_SLV3_REG: u8 = 0x10;
        pub const I2C_SLV3_CTRL: u8 = 0x11;
        pub const I2C_SLV3_DO: u8 = 0x12;
        pub const I2C_SLV4_ADDR: u8 = 0x13;
        pub const I2C_SLV4_REG: u8 = 0x14;
        pub const I2C_SLV4_CTRL: u8 = 0x15;
        pub const I2C_SLV4_DO: u8 = 0x16;
        pub const I2C_SLV4_DI: u8 = 0x17;
        
        // Selección de banco de registros
        pub const REG_BANK_SEL: u8 = 0x7F;
    }
}

/// Registros del DMP (Digital Motion Processor)
pub mod dmp {
    // Data output control registers
    pub const DATA_OUT_CTL1: u16 = 4 * 16;
    pub const DATA_OUT_CTL2: u16 = 4 * 16 + 2;
    pub const DATA_INTR_CTL: u16 = 4 * 16 + 12;
    pub const MOTION_EVENT_CTL: u16 = 4 * 16 + 14;
    pub const DATA_RDY_STATUS: u16 = 8 * 16 + 10;
    pub const FIFO_WATERMARK: u16 = 31 * 16 + 14;

    // Sensor ODR addresses
    pub const ODR_ACCEL: u16 = 11 * 16 + 14;
    pub const ODR_GYRO: u16 = 11 * 16 + 10;
    pub const ODR_CPASS: u16 = 11 * 16 + 6;
    pub const ODR_ALS: u16 = 11 * 16 + 2;
    pub const ODR_QUAT6: u16 = 10 * 16 + 12;
    pub const ODR_QUAT9: u16 = 10 * 16 + 8;
    pub const ODR_PQUAT6: u16 = 10 * 16 + 4;
    pub const ODR_GEOMAG: u16 = 10 * 16 + 0;
    pub const ODR_PRESSURE: u16 = 11 * 16 + 12;
    pub const ODR_GYRO_CALIBR: u16 = 11 * 16 + 8;
    pub const ODR_CPASS_CALIBR: u16 = 11 * 16 + 4;

    // Batch mode
    pub const BM_BATCH_CNTR: u16 = 27 * 16;
    pub const BM_BATCH_THLD: u16 = 19 * 16 + 12;
    pub const BM_BATCH_MASK: u16 = 21 * 16 + 14;

    // Bias registers
    pub const ACCEL_BIAS_X: u16 = 110 * 16 + 4;
    pub const ACCEL_BIAS_Y: u16 = 110 * 16 + 8;
    pub const ACCEL_BIAS_Z: u16 = 110 * 16 + 12;
    pub const GYRO_BIAS_X: u16 = 139 * 16 + 4;
    pub const GYRO_BIAS_Y: u16 = 139 * 16 + 8;
    pub const GYRO_BIAS_Z: u16 = 139 * 16 + 12;
    pub const CPASS_BIAS_X: u16 = 126 * 16 + 4;
    pub const CPASS_BIAS_Y: u16 = 126 * 16 + 8;
    pub const CPASS_BIAS_Z: u16 = 126 * 16 + 12;

    // Scale and gain registers
    pub const GYRO_SF: u16 = 19 * 16;
    pub const ACCEL_ONLY_GAIN: u16 = 16 * 16 + 12;
    pub const ACCEL_ALPHA_VAR: u16 = 91 * 16;
    pub const ACCEL_A_VAR: u16 = 92 * 16;
    pub const ACCEL_CAL_RATE: u16 = 94 * 16 + 4;
    
    // Compass calibration
    pub const CPASS_TIME_BUFFER: u16 = 112 * 16 + 14;
    pub const CPASS_RADIUS_3D_THRESH_ANOMALY: u16 = 112 * 16 + 8;
    
    // Compass matrix
    pub const CPASS_MTX_00: u16 = 23 * 16;
    pub const CPASS_MTX_01: u16 = 23 * 16 + 4;
    pub const CPASS_MTX_02: u16 = 23 * 16 + 8;
    pub const CPASS_MTX_10: u16 = 23 * 16 + 12;
    pub const CPASS_MTX_11: u16 = 24 * 16;
    pub const CPASS_MTX_12: u16 = 24 * 16 + 4;
    pub const CPASS_MTX_20: u16 = 24 * 16 + 8;
    pub const CPASS_MTX_21: u16 = 24 * 16 + 12;
    pub const CPASS_MTX_22: u16 = 25 * 16;
    
    // Pedometer
    pub const PEDSTD_STEPCTR: u16 = 54 * 16;
    
    // Wake on Motion
    pub const WOM_ENABLE: u16 = 64 * 16 + 14;
    pub const WOM_STATUS: u16 = 64 * 16 + 6;
    pub const WOM_THRESHOLD: u16 = 64 * 16;
    pub const WOM_CNTR_TH: u16 = 64 * 16 + 12;
    
    // Gyro and Accel FSR
    pub const GYRO_FULLSCALE: u16 = 72 * 16 + 12;
    pub const ACC_SCALE: u16 = 30 * 16 + 0;
    pub const ACC_SCALE2: u16 = 79 * 16 + 4;
    
    // EIS authentication
    pub const EIS_AUTH_INPUT: u16 = 160 * 16 + 4;
    pub const EIS_AUTH_OUTPUT: u16 = 160 * 16 + 0;
    
    // BAC and B2S
    pub const BAC_RATE: u16 = 48 * 16 + 10;
    pub const B2S_RATE: u16 = 48 * 16 + 8;
    
    // B2S Matrix
    pub const B2S_MTX_00: u16 = 208 * 16;
    pub const B2S_MTX_01: u16 = 208 * 16 + 4;
    pub const B2S_MTX_02: u16 = 208 * 16 + 8;
    pub const B2S_MTX_10: u16 = 208 * 16 + 12;
    pub const B2S_MTX_11: u16 = 209 * 16;
    pub const B2S_MTX_12: u16 = 209 * 16 + 4;
    pub const B2S_MTX_20: u16 = 209 * 16 + 8;
    pub const B2S_MTX_21: u16 = 209 * 16 + 12;
    pub const B2S_MTX_22: u16 = 210 * 16;
    
    // Flip/Pickup
    pub const FP_RATE: u16 = 240 * 16 + 12;
    
    // Pedometer
    pub const PED_Y_RATIO: u16 = 17 * 16 + 0;
    
    // Orientation parameters
    pub const Q0_QUAT6: u16 = 33 * 16 + 0;
    pub const Q1_QUAT6: u16 = 33 * 16 + 4;
    pub const Q2_QUAT6: u16 = 33 * 16 + 8;
    pub const Q3_QUAT6: u16 = 33 * 16 + 12;
    
    // BAC states
    pub const BAC_STATE: u16 = 179 * 16 + 0;
    pub const BAC_STATE_PREV: u16 = 179 * 16 + 4;
    pub const BAC_ACT_ON: u16 = 182 * 16 + 0;
    pub const BAC_ACT_OFF: u16 = 183 * 16 + 0;
    pub const BAC_STILL_S_F: u16 = 177 * 16 + 0;
    pub const BAC_RUN_S_F: u16 = 177 * 16 + 4;
    pub const BAC_DRIVE_S_F: u16 = 178 * 16 + 0;
    pub const BAC_WALK_S_F: u16 = 178 * 16 + 4;
    pub const BAC_SMD_S_F: u16 = 178 * 16 + 8;
    pub const BAC_BIKE_S_F: u16 = 178 * 16 + 12;
    pub const BAC_E1_SHORT: u16 = 146 * 16 + 0;
    pub const BAC_E2_SHORT: u16 = 146 * 16 + 4;
    pub const BAC_E3_SHORT: u16 = 146 * 16 + 8;
    pub const BAC_VAR_RUN: u16 = 148 * 16 + 12;
    pub const BAC_TILT_INIT: u16 = 181 * 16 + 0;
    pub const BAC_MAG_ON: u16 = 225 * 16 + 0;
    pub const BAC_PS_ON: u16 = 74 * 16 + 0;
    pub const BAC_DRIVE_CONFIDENCE: u16 = 144 * 16 + 0;
    pub const BAC_WALK_CONFIDENCE: u16 = 144 * 16 + 4;
    pub const BAC_SMD_CONFIDENCE: u16 = 144 * 16 + 8;
    pub const BAC_BIKE_CONFIDENCE: u16 = 144 * 16 + 12;
    pub const BAC_STILL_CONFIDENCE: u16 = 145 * 16 + 0;
    pub const BAC_RUN_CONFIDENCE: u16 = 145 * 16 + 4;
    pub const BAC_MODE_CNTR: u16 = 150 * 16;
    pub const BAC_STATE_T_PREV: u16 = 185 * 16 + 4;
    pub const BAC_ACT_T_ON: u16 = 184 * 16 + 0;
    pub const BAC_ACT_T_OFF: u16 = 184 * 16 + 4;
    pub const BAC_STATE_WRDBS_PREV: u16 = 185 * 16 + 8;
    pub const BAC_ACT_WRDBS_ON: u16 = 184 * 16 + 8;
    pub const BAC_ACT_WRDBS_OFF: u16 = 184 * 16 + 12;
    pub const BAC_ACT_ON_OFF: u16 = 190 * 16 + 2;
    pub const PREV_BAC_ACT_ON_OFF: u16 = 188 * 16 + 2;
    pub const BAC_CNTR: u16 = 48 * 16 + 2;
}

/// Registros del magnetómetro AK09916
pub mod compass {
    // Direcciones del magnetómetro
    pub const AK09916_I2C_ADDR: u8 = 0x0C;
    
    // Registros del AK09916
    pub const WHO_AM_I: u8 = 0x01;
    pub const ST1: u8 = 0x10;
    pub const HXL: u8 = 0x11;
    pub const HXH: u8 = 0x12;
    pub const HYL: u8 = 0x13;
    pub const HYH: u8 = 0x14;
    pub const HZL: u8 = 0x15;
    pub const HZH: u8 = 0x16;
    pub const ST2: u8 = 0x18;
    pub const CNTL2: u8 = 0x31;
    pub const CNTL3: u8 = 0x32;

    // Constantes de control
    pub const MODE_POWER_DOWN: u8 = 0x00;
    pub const MODE_SINGLE_MEASURE: u8 = 0x01;
    pub const MODE_CONTINUOUS_1: u8 = 0x02;
    pub const MODE_CONTINUOUS_2: u8 = 0x04;
    pub const MODE_CONTINUOUS_3: u8 = 0x06;
    pub const MODE_CONTINUOUS_4: u8 = 0x08;
    pub const MODE_SELF_TEST: u8 = 0x10;
    pub const AK09916_RESET: u8 = 0x01;
}

// Se ha movido la definición de registros de compass.rd a este módulo:
pub mod compass_rd {
    // AK09916 registers
    pub const AK09916_WIA1: u8 = 0x00;
    pub const AK09916_WIA2: u8 = 0x01;
    pub const AK09916_ST1: u8 = 0x10;
    pub const AK09916_HXL: u8 = 0x11;
    pub const AK09916_HXH: u8 = 0x12;
    pub const AK09916_HYL: u8 = 0x13;
    pub const AK09916_HYH: u8 = 0x14;
    pub const AK09916_HZL: u8 = 0x15;
    pub const AK09916_HZH: u8 = 0x16;
    pub const AK09916_ST2: u8 = 0x18;
    pub const AK09916_CNTL2: u8 = 0x31;
    pub const AK09916_CNTL3: u8 = 0x32;
    
    // AK09916 CNTL2 modes
    pub const AK09916_MODE_POWERDOWN: u8 = 0x00;
    pub const AK09916_MODE_SINGLE: u8 = 0x01;
    pub const AK09916_MODE_CONT_10HZ: u8 = 0x02;
    pub const AK09916_MODE_CONT_20HZ: u8 = 0x04;
    pub const AK09916_MODE_CONT_50HZ: u8 = 0x06;
    pub const AK09916_MODE_CONT_100HZ: u8 = 0x08;
}

/// Registros para magnetómetros AKM
pub mod ak_reg {
    // Registros comunes para todos los AK
    pub const WIA: u8 = 0x00;         // Identificación de dispositivo
    pub const INFO: u8 = 0x01;        // Información
    pub const STATUS: u8 = 0x02;      // Estado de datos
    pub const MEASURE_DATA: u8 = 0x03; // Inicio de los datos medidos
    pub const MODE: u8 = 0x0A;        // Registro de modo
    pub const CNTL: u8 = 0x0A;        // Control (alias de MODE)
    pub const ASTC: u8 = 0x0C;        // Control de self-test
    
    // AK09916 específico
    pub const AK09916_STATUS1: u8 = 0x10;
    pub const AK09916_STATUS2: u8 = 0x18;
    pub const AK09916_CNTL2: u8 = 0x31;
    pub const AK09916_CNTL3: u8 = 0x32;
    pub const AK09916_MEASURE_DATA: u8 = 0x11;
    
    // Otras variantes de AK
    pub const AK09911_STATUS1: u8 = 0x10;
    pub const AK09911_CNTL2: u8 = 0x31;
    pub const AK09911_MEASURE_DATA: u8 = 0x11;
    
    pub const AK09912_STATUS1: u8 = 0x10;
    pub const AK09912_CNTL1: u8 = 0x30;
    pub const AK09912_CNTL2: u8 = 0x31;
    pub const AK09912_MEASURE_DATA: u8 = 0x11;
    
    pub const AK8963_CNTL1: u8 = 0x0A;
    
    // Sensibilidad
    pub const AK09911_SENSITIVITY: u8 = 0x60;
    pub const AK09912_SENSITIVITY: u8 = 0x60;
    pub const AK8963_SENSITIVITY: u8 = 0x10;
}

/// Constantes para los canales secundarios I2C
pub mod slave_reg {
    pub const I2C_SLV0_ADDR: u8 = 0x03;
    pub const I2C_SLV0_REG: u8 = 0x04;
    pub const I2C_SLV0_CTRL: u8 = 0x05;
    pub const I2C_SLV0_DO: u8 = 0x06;
    pub const I2C_SLV1_ADDR: u8 = 0x07;
    pub const I2C_SLV1_REG: u8 = 0x08;
    pub const I2C_SLV1_CTRL: u8 = 0x09;
    pub const I2C_SLV1_DO: u8 = 0x0A;
    pub const I2C_SLV2_ADDR: u8 = 0x0B;
    pub const I2C_SLV2_REG: u8 = 0x0C;
    pub const I2C_SLV2_CTRL: u8 = 0x0D;
    pub const I2C_SLV2_DO: u8 = 0x0E;
    pub const I2C_SLV3_ADDR: u8 = 0x0F;
    pub const I2C_SLV3_REG: u8 = 0x10;
    pub const I2C_SLV3_CTRL: u8 = 0x11;
    pub const I2C_SLV3_DO: u8 = 0x12;
    pub const I2C_SLV4_ADDR: u8 = 0x13;
    pub const I2C_SLV4_REG: u8 = 0x14;
    pub const I2C_SLV4_CTRL: u8 = 0x15;
    pub const I2C_SLV4_DO: u8 = 0x16;
    pub const I2C_SLV4_DI: u8 = 0x17;
    
    pub const EXT_SENS_DATA_00: u8 = 0x3B;
    pub const I2C_MST_DELAY_CTRL: u8 = 0x02;
}
