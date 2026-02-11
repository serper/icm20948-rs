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
        pub const SINGLE_FIFO_PRIORITY_SEL: u8 = 0x26;
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

        pub const ACCEL_INTEL_CTRL: u8 = 0x12;
        pub const ACCEL_WOM_THR: u8 = 0x13;
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
    pub mod data_output_control {
        pub const DATA_OUT_CTL1: u16 = 4 * 16;
        pub const DATA_OUT_CTL2: u16 = 4 * 16 + 2;
        pub const DATA_INTR_CTL: u16 = 4 * 16 + 12;
        pub const MOTION_EVENT_CTL: u16 = 4 * 16 + 14;
        pub const DATA_RDY_STATUS: u16 = 8 * 16 + 10;
        pub const FIFO_WATERMARK: u16 = 31 * 16 + 14;
    }

    pub mod odr {
        pub const ACCEL: u16 = 11 * 16 + 14;
        pub const GYRO: u16 = 11 * 16 + 10;
        pub const CPASS: u16 = 11 * 16 + 6;
        pub const ALS: u16 = 11 * 16 + 2;
        pub const QUAT6: u16 = 10 * 16 + 12;
        pub const QUAT9: u16 = 10 * 16 + 8;
        pub const PQUAT6: u16 = 10 * 16 + 4;
        pub const GEOMAG: u16 = 10 * 16;
        pub const PRESSURE: u16 = 11 * 16 + 12;
        pub const GYRO_CALIBR: u16 = 11 * 16 + 8;
        pub const CPASS_CALIBR: u16 = 11 * 16 + 4;
    }

    // ODR counter registers
    pub mod odr_counter {
        pub const ACCEL: u16 = 9 * 16 + 14;
        pub const GYRO: u16 = 9 * 16 + 10;
        pub const CPASS: u16 = 9 * 16 + 6;
        pub const ALS: u16 = 9 * 16 + 2;
        pub const QUAT6: u16 = 8 * 16 + 12;
        pub const QUAT9: u16 = 8 * 16 + 8;
        pub const PQUAT6: u16 = 8 * 16 + 4;
        pub const GEOMAG: u16 = 8 * 16;
        pub const PRESSURE: u16 = 9 * 16 + 12;
        pub const GYRO_CALIBR: u16 = 9 * 16 + 8;
        pub const CPASS_CALIBR: u16 = 9 * 16 + 4;
    }

    // Batch mode
    pub mod bm_batch {
        pub const CNTR: u16 = 27 * 16;
        pub const THLD: u16 = 19 * 16 + 12;
        pub const MASK: u16 = 21 * 16 + 14;
    }

    // Bias registers
    pub mod bias {
        pub const ACCEL_X: u16 = 110 * 16 + 4;
        pub const ACCEL_Y: u16 = 110 * 16 + 8;
        pub const ACCEL_Z: u16 = 110 * 16 + 12;
        pub const GYRO_X: u16 = 139 * 16 + 4;
        pub const GYRO_Y: u16 = 139 * 16 + 8;
        pub const GYRO_Z: u16 = 139 * 16 + 12;
        pub const CPASS_X: u16 = 126 * 16 + 4;
        pub const CPASS_Y: u16 = 126 * 16 + 8;
        pub const CPASS_Z: u16 = 126 * 16 + 12;
    }

    // Scale and gain registers
    pub mod scale {
        pub const GYRO_SF: u16 = 19 * 16;
        pub const ACCEL_ONLY_GAIN: u16 = 16 * 16 + 12;
        pub const ACCEL_ALPHA_VAR: u16 = 91 * 16;
        pub const ACCEL_A_VAR: u16 = 92 * 16;
        pub const ACCEL_CAL_RATE: u16 = 94 * 16 + 4;
    }

    // Compass calibration
    pub mod compass_cal {
        pub const TIME_BUFFER: u16 = 112 * 16 + 14;
        pub const RADIUS_3D_THRESH_ANOMALY: u16 = 112 * 16 + 8;
    }

    // Compass matrix
    pub mod cpass {
        pub const MTX_00: u16 = 23 * 16;
        pub const MTX_01: u16 = 23 * 16 + 4;
        pub const MTX_02: u16 = 23 * 16 + 8;
        pub const MTX_10: u16 = 23 * 16 + 12;
        pub const MTX_11: u16 = 24 * 16;
        pub const MTX_12: u16 = 24 * 16 + 4;
        pub const MTX_20: u16 = 24 * 16 + 8;
        pub const MTX_21: u16 = 24 * 16 + 12;
        pub const MTX_22: u16 = 25 * 16;
    }

    // Pedometer
    pub mod pedometer {
        pub const BP_B: u16 = 49 * 16 + 12;
        pub const BP_A4: u16 = 52 * 16;
        pub const BP_A3: u16 = 52 * 16 + 4;
        pub const BP_A2: u16 = 52 * 16 + 8;
        pub const BP_A1: u16 = 52 * 16 + 12;
        pub const SB: u16 = 50 * 16 + 8;
        pub const SB_TIME: u16 = 50 * 16 + 12;
        pub const PEAKTHRSH: u16 = 57 * 16 + 8;
        pub const TIML: u16 = 50 * 16 + 10;
        pub const TIMH: u16 = 50 * 16 + 14;
        pub const PEAK: u16 = 57 * 16 + 4;
        pub const STEPCTR: u16 = 54 * 16;
        pub const STEPCTR2: u16 = 58 * 16 + 8;
        pub const TIMECTR: u16 = 60 * 16 + 4;
        pub const DECI: u16 = 58 * 16;
        pub const SB2: u16 = 60 * 16 + 14;
        pub const STPDET_TIMESTAMP: u16 = 18 * 16 + 8;
        pub const PEDSTEP_IND: u16 = 19 * 16 + 4;
        pub const PED_Y_RATIO: u16 = 17 * 16;
    }
    // Wake on Motion
    pub mod wom {
        pub const ENABLE: u16 = 64 * 16 + 14;
        pub const STATUS: u16 = 64 * 16 + 6;
        pub const THRESHOLD: u16 = 64 * 16;
        pub const CNTR_TH: u16 = 64 * 16 + 12;
    }

    // Gyro and Accel FSR
    pub mod fsr {
        pub const GYRO: u16 = 72 * 16 + 12;
        pub const ACC: u16 = 30 * 16;
        pub const ACC2: u16 = 79 * 16 + 4;
    }

    // EIS authentication
    pub mod eis_auth {
        pub const INPUT: u16 = 160 * 16 + 4;
        pub const OUTPUT: u16 = 160 * 16;
    }

    // BAC and B2S
    pub mod bac {
        pub const BAC_RATE: u16 = 48 * 16 + 10;
        pub const B2S_RATE: u16 = 48 * 16 + 8;
    }

    // B2S Matrix
    pub mod b2s_mtx {
        pub const B2S_MTX_00: u16 = 208 * 16;
        pub const B2S_MTX_01: u16 = 208 * 16 + 4;
        pub const B2S_MTX_02: u16 = 208 * 16 + 8;
        pub const B2S_MTX_10: u16 = 208 * 16 + 12;
        pub const B2S_MTX_11: u16 = 209 * 16;
        pub const B2S_MTX_12: u16 = 209 * 16 + 4;
        pub const B2S_MTX_20: u16 = 209 * 16 + 8;
        pub const B2S_MTX_21: u16 = 209 * 16 + 12;
        pub const B2S_MTX_22: u16 = 210 * 16;
    }

    // Orientation parameters
    pub mod orientation {
        pub const Q0_QUAT6: u16 = 33 * 16;
        pub const Q1_QUAT6: u16 = 33 * 16 + 4;
        pub const Q2_QUAT6: u16 = 33 * 16 + 8;
        pub const Q3_QUAT6: u16 = 33 * 16 + 12;
    }

    // 9-axis thresholds
    pub mod nine_axis_thr {
        pub const MAGN_THR_9X: u16 = 80 * 16;
        pub const MAGN_LPF_THR_9X: u16 = 80 * 16 + 8;
        pub const QFB_THR_9X: u16 = 80 * 16 + 12;
    }

    // BAC states
    pub mod bac_state {
        pub const STATE: u16 = 179 * 16;
        pub const STATE_PREV: u16 = 179 * 16 + 4;
        pub const ACT_ON: u16 = 182 * 16;
        pub const ACT_OFF: u16 = 183 * 16;
        pub const STILL_S_F: u16 = 177 * 16;
        pub const RUN_S_F: u16 = 177 * 16 + 4;
        pub const DRIVE_S_F: u16 = 178 * 16 + 0;
        pub const WALK_S_F: u16 = 178 * 16 + 4;
        pub const SMD_S_F: u16 = 178 * 16 + 8;
        pub const BIKE_S_F: u16 = 178 * 16 + 12;
        pub const E1_SHORT: u16 = 146 * 16 + 0;
        pub const E2_SHORT: u16 = 146 * 16 + 4;
        pub const E3_SHORT: u16 = 146 * 16 + 8;
        pub const VAR_RUN: u16 = 148 * 16 + 12;
        pub const TILT_INIT: u16 = 181 * 16 + 0;
        pub const MAG_ON: u16 = 225 * 16 + 0;
        pub const PS_ON: u16 = 74 * 16 + 0;
        pub const DRIVE_CONFIDENCE: u16 = 144 * 16 + 0;
        pub const WALK_CONFIDENCE: u16 = 144 * 16 + 4;
        pub const SMD_CONFIDENCE: u16 = 144 * 16 + 8;
        pub const BIKE_CONFIDENCE: u16 = 144 * 16 + 12;
        pub const STILL_CONFIDENCE: u16 = 145 * 16 + 0;
        pub const RUN_CONFIDENCE: u16 = 145 * 16 + 4;
        pub const MODE_CNTR: u16 = 150 * 16;
        pub const STATE_T_PREV: u16 = 185 * 16 + 4;
        pub const ACT_T_ON: u16 = 184 * 16 + 0;
        pub const ACT_T_OFF: u16 = 184 * 16 + 4;
        pub const STATE_WRDBS_PREV: u16 = 185 * 16 + 8;
        pub const ACT_WRDBS_ON: u16 = 184 * 16 + 8;
        pub const ACT_WRDBS_OFF: u16 = 184 * 16 + 12;
        pub const ACT_ON_OFF: u16 = 190 * 16 + 2;
        pub const PREV_ACT_ON_OFF: u16 = 188 * 16 + 2;
        pub const CNTR: u16 = 48 * 16 + 2;
    }

    pub mod data_ready_status {
        pub const GYRO: u16 = 0x0001; // Gyro samples available
        pub const ACCEL: u16 = 0x0002; // Accel samples available
        pub const SECONDARY_COMPASS: u16 = 0x0008; // Secondary compass samples available
    }

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

    /// Output mask bits for data_output_control1
    pub mod output_mask {
        /// ACCEL_SET
        pub const ACCEL: u16 = 0x8000;
        /// GYRO_SET
        pub const GYRO: u16 = 0x4000;
        /// CPASS_SET
        pub const CPASS: u16 = 0x2000;
        /// ALS_SET
        pub const ALS: u16 = 0x1000;
        /// QUAT6_SET
        pub const QUAT6: u16 = 0x0800;
        /// QUAT9_SET
        pub const QUAT9: u16 = 0x0400;
        /// PQUAT6_SET
        pub const PQUAT6: u16 = 0x0200;
        /// GEOMAG_SET
        pub const GEOMAG: u16 = 0x0100;
        /// PRESSURE_SET
        pub const PRESSURE: u16 = 0x0080;
        /// GYRO_CALIBR_SET
        pub const GYRO_CALIBR: u16 = 0x0040;
        /// CPASS_CALIBR_SET
        pub const CPASS_CALIBR: u16 = 0x0020;
        /// PED_STEPDET_SET
        pub const PED_STEPDET: u16 = 0x0010;
        /// HEADER2_SET
        pub const HEADER2: u16 = 0x0008;
        /// PED_STEPIND_SET
        pub const PED_STEPIND: u16 = 0x0007;
    }

    /// Output mask bits for data_output_control2
    pub mod output_mask2 {
        pub const SCREEN_ROTATION: u16 = 0x0020;
        pub const SECONDARY_ON_OFF: u16 = 0x0040;
        pub const ACTIVITY_RECOGNITION_BAC: u16 = 0x0080;
        pub const BATCH_MODE_ENABLE: u16 = 0x0100;
        pub const PICKUP: u16 = 0x0400;
        pub const FSYNC_DETECTION: u16 = 0x0800;
        pub const COMPASS_ACCURACY: u16 = 0x1000;
        pub const GYRO_ACCURACY: u16 = 0x2000;
        pub const ACCEL_ACCURACY: u16 = 0x4000;
    }

    /// Motion event control mask bits
    pub mod motion_event_control {
        /// BAC_WEAR_EN
        pub const BAC_WEAR_EN: u16 = 0x8000;
        /// PEDOMETER_EN
        pub const PEDOMETER_EN: u16 = 0x4000;
        /// PEDOMETER_INT_EN
        pub const PEDOMETER_INT_EN: u16 = 0x2000;
        /// SMD_EN
        pub const SMD_EN: u16 = 0x0800;
        /// BTS_EN (Bring-To-See)
        pub const BTS_EN: u16 = 0x0020;
        /// FLIP_PICKUP_EN
        pub const FLIP_PICKUP_EN: u16 = 0x0010;
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

    // Activity Recognition
    pub mod activity_recognition {
        pub const RATE: u16 = 48 * 16 + 10;
        pub const STATE: u16 = 179 * 16;
        pub const STATE_PREV: u16 = 179 * 16 + 4;
        pub const ACT_ON: u16 = 182 * 16;
        pub const ACT_OFF: u16 = 183 * 16;
        pub const STILL_S_F: u16 = 177 * 16;
        pub const RUN_S_F: u16 = 177 * 16 + 4;
        pub const DRIVE_S_F: u16 = 178 * 16 + 0;
        pub const WALK_S_F: u16 = 178 * 16 + 4;
        pub const SMD_S_F: u16 = 178 * 16 + 8;
        pub const BIKE_S_F: u16 = 178 * 16 + 12;
        pub const E1_SHORT: u16 = 146 * 16 + 0;
        pub const E2_SHORT: u16 = 146 * 16 + 4;
        pub const E3_SHORT: u16 = 146 * 16 + 8;
        pub const VAR_RUN: u16 = 148 * 16 + 12;
        pub const TILT_INIT: u16 = 181 * 16 + 0;
        pub const MAG_ON: u16 = 225 * 16 + 0;
        pub const PS_ON: u16 = 74 * 16 + 0;
        pub const BIKE_PREFERENCE: u16 = 173 * 16 + 8;
        pub const MAG_I2C_ADDR: u16 = 229 * 16 + 8;
        pub const PS_I2C_ADDR: u16 = 75 * 16 + 4;
        pub const DRIVE_CONFIDENCE: u16 = 144 * 16 + 0;
        pub const WALK_CONFIDENCE: u16 = 144 * 16 + 4;
        pub const SMD_CONFIDENCE: u16 = 144 * 16 + 8;
        pub const BIKE_CONFIDENCE: u16 = 144 * 16 + 12;
        pub const STILL_CONFIDENCE: u16 = 145 * 16 + 0;
        pub const RUN_CONFIDENCE: u16 = 145 * 16 + 4;
        pub const MODE_CNTR: u16 = 150 * 16;
        pub const STATE_T_PREV: u16 = 185 * 16 + 4;
        pub const ACT_T_ON: u16 = 184 * 16 + 0;
        pub const ACT_T_OFF: u16 = 184 * 16 + 4;
        pub const STATE_WRDBS_PREV: u16 = 185 * 16 + 8;
        pub const ACT_WRDBS_ON: u16 = 184 * 16 + 8;
        pub const ACT_WRDBS_OFF: u16 = 184 * 16 + 12;
        pub const ACT_ON_OFF: u16 = 190 * 16 + 2;
        pub const PREV_ACT_ON_OFF: u16 = 188 * 16 + 2;
        pub const CNTR: u16 = 48 * 16 + 2;
    }

    // DMP running counter
    pub mod dmp_running_counter {
        pub const DMPRATE_CNTR: u16 = 18 * 16 + 4;
    }

    // SMD
    pub mod smd {
        pub const SMD_VAR_TH: u16 = 141 * 16 + 12;
        pub const SMD_VAR_TH_DRIVE: u16 = 143 * 16 + 12;
        pub const SMD_DRIVE_TIMER_TH: u16 = 143 * 16 + 8;
        pub const SMD_TILT_ANGLE_TH: u16 = 179 * 16 + 12;
        pub const BAC_SMD_ST_TH: u16 = 179 * 16 + 8;
        pub const BAC_ST_ALPHA4: u16 = 180 * 16 + 12;
        pub const BAC_ST_ALPHA4A: u16 = 176 * 16 + 12;
    }

    // Wake on Motion
    pub mod wake_on_motion {
        pub const WOM_ENABLE: u16 = 64 * 16 + 14;
        pub const WOM_STATUS: u16 = 64 * 16 + 6;
        pub const WOM_THRESHOLD_DMP: u16 = 64 * 16;
        pub const WOM_CNTR_TH: u16 = 64 * 16 + 12;
    }

    // Flip/Pick-up
    pub mod flip_pickup {
        pub const VAR_ALPHA: u16 = 245 * 16 + 8;
        pub const STILL_TH: u16 = 246 * 16 + 4;
        pub const MID_STILL_TH: u16 = 244 * 16 + 8;
        pub const NOT_STILL_TH: u16 = 246 * 16 + 8;
        pub const VIB_REJ_TH: u16 = 241 * 16 + 8;
        pub const MAX_PICKUP_T_TH: u16 = 244 * 16 + 12;
        pub const PICKUP_TIMEOUT_TH: u16 = 248 * 16 + 8;
        pub const STILL_CONST_TH: u16 = 246 * 16 + 12;
        pub const MOTION_CONST_TH: u16 = 240 * 16 + 8;
        pub const VIB_COUNT_TH: u16 = 242 * 16 + 8;
        pub const STEADY_TILT_TH: u16 = 247 * 16 + 8;
        pub const STEADY_TILT_UP_TH: u16 = 242 * 16 + 12;
        pub const Z_FLAT_TH_MINUS: u16 = 243 * 16 + 8;
        pub const Z_FLAT_TH_PLUS: u16 = 243 * 16 + 12;
        pub const DEV_IN_POCKET_TH: u16 = 76 * 16 + 12;
        pub const PICKUP_CNTR: u16 = 247 * 16 + 4;
        pub const RATE: u16 = 240 * 16 + 12;
    }

    pub enum AlgoFreq {
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
    pub enum AccelCalParam {
        /// AlphaVar
        AlphaVar = 0,
        /// AVar
        AVar = 1,
        /// Div
        Div = 2,
    }
}

// /// Registros del magnetómetro AK09916
// pub mod compass {
//     // Direcciones del magnetómetro
//     pub const AK09916_I2C_ADDR: u8 = 0x0C;

//     // Registros del AK09916
//     pub const WHO_AM_I: u8 = 0x01;
//     pub const ST1: u8 = 0x10;
//     pub const HXL: u8 = 0x11;
//     pub const HXH: u8 = 0x12;
//     pub const HYL: u8 = 0x13;
//     pub const HYH: u8 = 0x14;
//     pub const HZL: u8 = 0x15;
//     pub const HZH: u8 = 0x16;
//     pub const ST2: u8 = 0x18;
//     pub const CNTL2: u8 = 0x31;
//     pub const CNTL3: u8 = 0x32;

//     // Constantes de control
//     pub const MODE_POWER_DOWN: u8 = 0x00;
//     pub const MODE_SINGLE_MEASURE: u8 = 0x01;
//     pub const MODE_CONTINUOUS_1: u8 = 0x02;
//     pub const MODE_CONTINUOUS_2: u8 = 0x04;
//     pub const MODE_CONTINUOUS_3: u8 = 0x06;
//     pub const MODE_CONTINUOUS_4: u8 = 0x08;
//     pub const MODE_SELF_TEST: u8 = 0x10;
//     pub const AK09916_RESET: u8 = 0x01;
// }

/// Registros para magnetómetros AKM
pub mod ak_reg {
    // Registros comunes para todos los AK
    pub const WIA: u8 = 0x00; // Identificación de dispositivo
    pub const INFO: u8 = 0x01; // Información
    pub const STATUS: u8 = 0x02; // Estado de datos
    pub const MEASURE_DATA: u8 = 0x03; // Inicio de los datos medidos
    pub const MODE: u8 = 0x0A; // Registro de modo
    pub const CNTL: u8 = 0x0A; // Control (alias de MODE)
    pub const ASTC: u8 = 0x0C; // Control de self-test

    // AK09916 registers
    pub const AK09916_HXL: u8 = 0x11;
    pub const AK09916_HXH: u8 = 0x12;
    pub const AK09916_HYL: u8 = 0x13;
    pub const AK09916_HYH: u8 = 0x14;
    pub const AK09916_HZL: u8 = 0x15;
    pub const AK09916_HZH: u8 = 0x16;

    // AK09916 específico
    pub const AK09916_STATUS1: u8 = 0x10;
    pub const AK09916_STATUS2: u8 = 0x18;
    pub const AK09916_DMP_READ: u8 = 0x03;
    pub const AK09916_CNTL1: u8 = 0x30;
    pub const AK09916_CNTL2: u8 = 0x31;
    pub const AK09916_CNTL3: u8 = 0x32;
    pub const AK09916_MEASURE_DATA: u8 = 0x11;

    // Otras variantes de AK
    pub const AK09911_STATUS1: u8 = 0x10;
    pub const AK09911_STATUS2: u8 = 0x18;
    pub const AK09911_DMP_READ: u8 = 0x03;
    pub const AK09911_CNTL1: u8 = 0x30;
    pub const AK09911_CNTL2: u8 = 0x31;
    pub const AK09911_CNTL3: u8 = 0x32;
    pub const AK09911_MEASURE_DATA: u8 = 0x11;

    pub const AK09912_STATUS1: u8 = 0x10;
    pub const AK09912_STATUS2: u8 = 0x18;
    pub const AK09912_DMP_READ: u8 = 0x03;
    pub const AK09912_CNTL1: u8 = 0x30;
    pub const AK09912_CNTL2: u8 = 0x31;
    pub const AK09912_CNTL3: u8 = 0x32;
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
