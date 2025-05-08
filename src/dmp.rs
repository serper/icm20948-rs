//! DMP implementation for ICM20948

use crate::device::{Icm20948, Icm20948Error, INV_NEEDS_ACCEL_MASK, INV_NEEDS_GYRO_MASK, INV_NEEDS_COMPASS_MASK};
use crate::compass;
use crate::dmp3_firmware;
use crate::types::{GyroFullScale, AccelFullScale, bits, OdrSensor, Sensor, AndroidSensor, SENSORTOCONTROLBITS, 
    DMP_LOAD_START, DMP_START_ADDRESS};
use crate::register::registers::{bank0, bank1, bank2, bank3};
use crate::register::dmp::{self};
use crate::interface::Interface;
use embedded_hal::blocking::delay::DelayMs;

/// Driver para el DMP (Digital Motion Processor) del ICM20948
pub struct DmpDriverIcm20948<I, D> {
    pub(crate) device: Icm20948<I, D>,
}

impl<I, D, E> DmpDriverIcm20948<I, D>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
{
    /// Creates a new `DmpDriverIcm20948`.
    pub fn new(device: Icm20948<I, D>) -> Self {
        Self { device }
    }

    /// Initializes the DMP driver.
    pub fn initialize(&mut self) -> Result<(), Icm20948Error> {
        // 0. Setup I2C controller for magnetometer
        // Intentar activar el magnetómetro
        self.device.setup_compass(compass::CompassType::AK09916, 0x0C)?;

        // 1. Configurar el I2C auxiliar para el magnetómetr
        // Para I2C_SLV0 - Leer los 10 bytes desde RSV2
        self.i2c_controller_configure_peripheral(0, 0x0C, 0x03, 10, true, true, false, true, true, None)?;

        // Para I2C_SLV1 - Configurar el modo de medición única
        self.i2c_controller_configure_peripheral(1, 0x0C, 0x31, 1, false, true, false, false, false, Some(0x01))?;


        // 2. Configurar el reloj y los modos de energía
        let mst_odr_config = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
        self.device.write_reg::<bank3::Bank>(bank3::I2C_MST_ODR_CONFIG, mst_odr_config)?;  

        self.device.set_clock_source(bits::CLK_PLL)?; // Set the clock source to PLL
        
        // 3. Activar acelerómetro y giroscopio, desactivar sensor de presión y magnetómetro (accesible por DMP)
        self.device.write_reg::<bank0::Bank>(bank0::PWR_MGMT_2, 0x40)?;
        // self.device.enter_i2c_low_power_mode()?; // Enter low power mode for I2C
        self.device.set_lowpower_or_highperformance(0)?; // Set to low power mode
        // Wait 1ms for the I2C to stabilize
        self.device.delay.delay_ms(1u32);
        
        // 4. Deshabilitar FIFO y DMP
        self.enable_dmp(false)?;
        self.device.set_fifo_enable(false)?;
        
        // 5. Configurar el rango de escala del acelerómetro y giroscopio
        self.device.set_accel_fullscale(AccelFullScale::Fs4G)?;
        self.device.set_gyro_fullscale(GyroFullScale::Fs2000Dps)?;
        self.device.enable_gyro_dlpf(true)?; // Habilitar DLPF para el giroscopio
        
        self.device.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_1, 0x00)?;
        self.device.write_mems_reg::<bank0::Bank>(bank0::FIFO_EN_2, 0x00)?;
        self.device.modify_reg::<bank0::Bank,_>(bank0::INT_ENABLE_1, |val| val & !bits::INT_RAW_DATA_RDY_EN)?;
        
        self.device.reset_fifo()?;

        self.device.set_accel_divider(4)?; // 4ms = 250Hz
        self.device.set_gyro_divider(4)?; // 4ms = 250Hz

        // 6. Configurar dirección de inicio del DMP y cargar firmware
        let start_address = self.get_dmp_start_address();
        self.device.write_mems_regs::<bank2::Bank>(bank2::PRGM_START_ADDRH, &[
            (start_address >> 8) as u8,
            (start_address & 0xFF) as u8,
        ])?;
        
        self.load_firmware(dmp3_firmware::DMP3_FIRMWARE)?; // Cargar firmware DMP3

        self.device.write_mems_regs::<bank2::Bank>(bank2::PRGM_START_ADDRH, &[
            (start_address >> 8) as u8,
            (start_address & 0xFF) as u8,
        ])?; // Volver a escribir la dirección de inicio del DMP

        self.device.write_mems_reg::<bank0::Bank>(bank0::HW_FIX_DISABLE,0x48)?;

        self.device.write_mems_reg::<bank0::Bank>(bank0::SINGLE_FIFO_PRIORITY_SEL, 0xE4)?;

        // 7. Configurar matrices de transformación y escalas
        self.device.write_mems(dmp::fsr::ACC, &[0x04, 0x00, 0x00, 0x00])?;
        self.device.write_mems(dmp::fsr::ACC2, &[0x00, 0x04, 0x00, 0x00])?;

        // Values from Invensense Nucleo firmware
        self.set_compass_matrix(&[ 0x09999999, 0x00000000, 0x00000000, 
                                                0x00000000, 0xF6666667, 0x00000000,
                                                0x00000000, 0x00000000, 0xF6666667])?;

        self.set_b2s_matrix(&[ 0x40000000, 0x00000000, 0x00000000, 
                                        0x00000000, 0x40000000, 0x00000000,
                                        0x00000000, 0x00000000, 0x40000000])?;

        self.dmp_set_gyro_sf(0x04, 0x03)?; // Set gyro scale factor to 4 (2000dps)
        self.device.write_mems(dmp::fsr::GYRO, &[0x10, 0x00, 0x00, 0x00])?;

        self.set_accel_feedback_gain(0x00E8BA2E)?;
        self.set_accel_cal_params(&[0x3D27D27D, 0x02D82D83, 0x0000])?;
        self.set_compass_cal_params(0x0045, None)?;
        
        Ok(())
    }
    
    /// Configurar el I2C auxiliar para periféricos
    fn i2c_controller_configure_peripheral(
        &mut self,
        slave_num: u8,
        i2c_addr: u8,
        reg_addr: u8,
        num_bytes: u8,
        read_not_write: bool,
        enable: bool,
        reg_dis: bool,
        group: bool,
        byte_swapped: bool,
        data_out: Option<u8>,
    ) -> Result<(), Icm20948Error> {
        // Determinar los registros a utilizar según el periférico seleccionado
        let (periph_addr_reg, periph_reg_reg, periph_ctrl_reg, periph_do_reg) = match slave_num {
            0 => (bank3::I2C_SLV0_ADDR, bank3::I2C_SLV0_REG, bank3::I2C_SLV0_CTRL, bank3::I2C_SLV0_DO),
            1 => (bank3::I2C_SLV1_ADDR, bank3::I2C_SLV1_REG, bank3::I2C_SLV1_CTRL, bank3::I2C_SLV1_DO),
            2 => (bank3::I2C_SLV2_ADDR, bank3::I2C_SLV2_REG, bank3::I2C_SLV2_CTRL, bank3::I2C_SLV2_DO),
            3 => (bank3::I2C_SLV3_ADDR, bank3::I2C_SLV3_REG, bank3::I2C_SLV3_CTRL, bank3::I2C_SLV3_DO),
            4 => (bank3::I2C_SLV4_ADDR, bank3::I2C_SLV4_REG, bank3::I2C_SLV4_CTRL, bank3::I2C_SLV4_DO),
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        // Configurar la dirección del periférico y la bandera RW
        let addr_val = if read_not_write {
            i2c_addr | 0x80 // Establecer bit RNW (bit 7)
        } else {
            i2c_addr & 0x7F // Asegurar que el bit RNW está limpio
        };
        
        // Escribir la dirección del periférico
        self.device.write_reg::<bank3::Bank>(periph_addr_reg, addr_val)?;

        // Si es una operación de escritura, configurar el registro DO (Data Out)
        if !read_not_write && slave_num <= 3 {
            if let Some(data_out) = data_out {
                self.device.write_reg::<bank3::Bank>(periph_do_reg, data_out)?;  
            } else {
                self.device.write_reg::<bank3::Bank>(periph_do_reg, 0)?;
            }
        }

        // Configurar el registro del periférico (subdireccion)
        self.device.write_reg::<bank3::Bank>(periph_reg_reg, reg_addr)?;
        
        // Configurar el registro de control
        let mut ctrl_val: u8 = 0;
        
        // Configurar los bits de control
        if enable {
            ctrl_val |= 0x80;                   // EN - bit 7: habilitar
        }
        
        if byte_swapped {
            ctrl_val |= 0x40;                   // BYTE_SW - bit 6: intercambiar bytes
        }
        
        if reg_dis {
            ctrl_val |= 0x20;                   // REG_DIS - bit 5: deshabilitar escritura de registro
        }
        
        if group {
            ctrl_val |= 0x10;                   // GRP - bit 4: agrupar
        }
        
        ctrl_val |= num_bytes & 0x0F;           // LENG - bits 0-3: longitud de datos (0-15 bytes)

        // Escribir el registro de control
        self.device.write_reg::<bank3::Bank>(periph_ctrl_reg, ctrl_val)?;
        
        Ok(())
    }

    /// Habilitar/Deshabilitar el DMP
    pub fn enable_dmp(&mut self, enable: bool) -> Result<(), Icm20948Error> {
        let mut user_ctrl = self.device.read_mems_reg::<bank0::Bank>(bank0::USER_CTRL)?;
        if enable {
            user_ctrl |= bits::DMP_EN; // DMP_EN bit
        } else {
            user_ctrl &= !bits::DMP_EN; // Limpiar DMP_EN bit
        }
        self.device.write_mems_reg::<bank0::Bank>(bank0::USER_CTRL, user_ctrl)?;
        Ok(())
    }

    /// Reset DMP
    pub fn reset_dmp(&mut self) -> Result<(), Icm20948Error> {
        self.device.modify_mems_reg::<bank0::Bank, _>(bank0::USER_CTRL, |val| val | bits::DMP_RST)?;
        self.device.delay.delay_ms(10u32); // Esperar 10ms
        self.device.modify_mems_reg::<bank0::Bank, _>(bank0::USER_CTRL, |val| val & !bits::DMP_RST)?;
        Ok(())
    }

    /// Verificar si el firmware está cargado
    pub fn is_firmware_loaded(&self) -> bool {
        self.device.is_firmware_loaded()
    }
    
    /// Load DMP firmware
    pub fn load_firmware(&mut self, firmware_data: &[u8]) -> Result<(), Icm20948Error> {
        // Write DMP firmware to memory
        self.device.write_mems(DMP_LOAD_START, firmware_data)?;

        // Verify DMP firmware in memory
        let mut data_cmp = vec![0u8; firmware_data.len()];
        self.device.read_mems(DMP_LOAD_START, &mut data_cmp)?;
        if firmware_data != &data_cmp[..] {
            return Err(Icm20948Error::FirmwareVerificationFailed);
        }

        // Set firmware loaded flag
        self.device.base_state.firmware_loaded = true;
        
        Ok(())
    }

    /// Get DMP start address
    pub fn get_dmp_start_address(&mut self) -> u16 {
        DMP_START_ADDRESS
    }

    /// Reset DMP control registers
    pub fn reset_control_registers(&mut self) -> Result<(), Icm20948Error> {
        // Reset all control registers to zero
        let zeros = [0u8, 0u8, 0u8, 0u8];
        
        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL1, &zeros)?;
        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL2, &zeros)?;
        self.device.write_mems(dmp::data_output_control::DATA_INTR_CTL, &zeros)?;
        self.device.write_mems(dmp::data_output_control::MOTION_EVENT_CTL, &zeros)?;
        self.device.write_mems(dmp::data_output_control::DATA_RDY_STATUS, &zeros)?;
        
        Ok(())
    }
    
    /// Enable/Disable DMP Sensors
    /// Set/unset the corresponding bits for the sensors in the DATA_OUT_CTL1 ,DATA_OUT_CTL2 ,DATA_RDY_STATUS ,MOTION_EVENT_CTL
    pub fn enable_dmp_sensor(&mut self, sensor: Sensor, enable: bool) -> Result<(), Icm20948Error> {
        let mut data_rdy_status = 0u16;
        let mut event_control = 0u16;
        
        let android_sensor = DmpDriverIcm20948::<I, D>::sensor_type_2_android_sensor(sensor);
        if (android_sensor as u8) >= (AndroidSensor::NumMax as u8) {
            return Err(Icm20948Error::InvalidParameter);
        }
        
        let mut delta = SENSORTOCONTROLBITS[android_sensor as usize];
        if delta == 0xFFFF {
            return Err(Icm20948Error::InvalidParameter);
        }

        let mut android_sensor_as_bit_mask: u32;
        if (android_sensor as u8) < 32 {
            android_sensor_as_bit_mask = 1 << (android_sensor as u8);
            if enable {
                self.device.base_state.enabled_sensors_0 |= android_sensor_as_bit_mask;
            } else {
                self.device.base_state.enabled_sensors_0 &= !(android_sensor_as_bit_mask);
            }
        } else {
            android_sensor_as_bit_mask = 1 << (android_sensor as u8 - 32);
            if enable {
                self.device.base_state.enabled_sensors_1 |= android_sensor_as_bit_mask;
            } else {
                self.device.base_state.enabled_sensors_1 &= !(android_sensor_as_bit_mask);
            }
        }
        
        delta = 0;

        for i in 0..32 {
            android_sensor_as_bit_mask = 1 << i;
            if (self.device.base_state.enabled_sensors_0 & android_sensor_as_bit_mask) != 0 {
                delta |= SENSORTOCONTROLBITS[i as usize];
            }
            if (self.device.base_state.enabled_sensors_1 & android_sensor_as_bit_mask) != 0 {
                delta |= SENSORTOCONTROLBITS[(i + 32) as usize];
            }

            if ((self.device.base_state.enabled_sensors_0 & android_sensor_as_bit_mask) & INV_NEEDS_ACCEL_MASK != 0) ||
                ((self.device.base_state.enabled_sensors_1 & android_sensor_as_bit_mask) & INV_NEEDS_ACCEL_MASK != 0) {
                // If the sensor requires accelerometer, enable it
                data_rdy_status |= dmp::data_ready_status::ACCEL;
                event_control |= dmp::motion_event_control::ACCEL_CAL_EN;
            }
            if ((self.device.base_state.enabled_sensors_0 & android_sensor_as_bit_mask) & INV_NEEDS_GYRO_MASK != 0) ||
                ((self.device.base_state.enabled_sensors_1 & android_sensor_as_bit_mask) & INV_NEEDS_GYRO_MASK != 0) {
                // If the sensor requires gyroscope, enable it
                data_rdy_status |= dmp::data_ready_status::GYRO;
                event_control |= dmp::motion_event_control::GYRO_CAL_EN;
            }
            if ((self.device.base_state.enabled_sensors_0 & android_sensor_as_bit_mask) & INV_NEEDS_COMPASS_MASK != 0) ||
                ((self.device.base_state.enabled_sensors_1 & android_sensor_as_bit_mask) & INV_NEEDS_COMPASS_MASK != 0) {
                // If the sensor requires compass, enable it
                data_rdy_status |= dmp::data_ready_status::SECONDARY_COMPASS;
                event_control |= dmp::motion_event_control::COMPASS_CAL_EN;
            }
        }

        self.device.set_sleep(false)?; // Desactivar el modo de suspensión
        self.device.low_power(false)?; // Desactivar el modo de bajo consumo

        let mut delta2 = 0u16;
        if (delta & dmp::output_mask::ACCEL as u16) != 0 {
            delta2 |= dmp::output_mask2::ACCEL_ACCURACY;
        }
        if (delta & dmp::output_mask::GYRO_CALIBR as u16) != 0 || (delta & dmp::output_mask::GYRO as u16) != 0 {
            delta2 |= dmp::output_mask2::GYRO_ACCURACY;
        }
        if (delta & dmp::output_mask::CPASS_CALIBR as u16) != 0 || (delta & dmp::output_mask::CPASS as u16) != 0 {   
            delta2 |= dmp::output_mask2::COMPASS_ACCURACY;
        }
        // TODO: Add other sensors as needed

        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL1, &delta.to_be_bytes())?;
        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL2, &delta2.to_be_bytes())?;
        self.device.write_mems(dmp::data_output_control::DATA_RDY_STATUS, &data_rdy_status.to_be_bytes())?;

        if delta & dmp::output_mask::QUAT9 as u16 != 0 {
            event_control |= dmp::motion_event_control::NINE_AXIS_EN;
        }
        if delta & dmp::output_mask::PED_STEPDET as u16 != 0 || 
            delta & dmp::output_mask::PED_STEPIND as u16 != 0 {
            event_control |= dmp::motion_event_control::PEDOMETER_EN;
        }
        if delta & dmp::output_mask::GEOMAG as u16 != 0 {
            event_control |= dmp::motion_event_control::GEOMAG_EN;
        }

        self.device.write_mems(dmp::data_output_control::MOTION_EVENT_CTL, &event_control.to_be_bytes())?;
        
        Ok(())
    }
    
    /// Set data output control register 1
    pub fn set_data_output_control1(&mut self, output_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = output_mask.to_be_bytes();
        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL1, &bytes)
    }
    
    /// Set data output control register 2
    pub fn set_data_output_control2(&mut self, output_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = output_mask.to_be_bytes();
        self.device.write_mems(dmp::data_output_control::DATA_OUT_CTL2, &bytes)
    }
    
    /// Set data interrupt control register
    pub fn set_data_interrupt_control(&mut self, interrupt_ctl: u32) -> Result<(), Icm20948Error> {
        // Only the lower 16 bits are used
        let bytes = (interrupt_ctl as u16).to_be_bytes();
        self.device.write_mems(dmp::data_output_control::DATA_INTR_CTL, &bytes)
    }
    
    /// Set FIFO watermark
    pub fn set_fifo_watermark(&mut self, fifo_wm: u16) -> Result<(), Icm20948Error> {
        let bytes = fifo_wm.to_be_bytes();
        self.device.write_mems(dmp::data_output_control::FIFO_WATERMARK, &bytes)
    }
    
    /// Set data ready status register
    pub fn set_data_rdy_status(&mut self, data_rdy: u16) -> Result<(), Icm20948Error> {
        let bytes = data_rdy.to_be_bytes();
        self.device.write_mems(dmp::data_output_control::DATA_RDY_STATUS, &bytes)
    }
    
    /// Set motion event control register
    pub fn set_motion_event_control(&mut self, motion_mask: u16) -> Result<(), Icm20948Error> {
        let bytes = motion_mask.to_be_bytes();
        self.device.write_mems(dmp::data_output_control::MOTION_EVENT_CTL, &bytes)
    }
    
    /// Set sensor rate
    pub fn set_sensor_rate(&mut self, sensor: OdrSensor, divider: i16) -> Result<(), Icm20948Error> {
        let odr_addr = match sensor {
            OdrSensor::Accel => dmp::odr::ACCEL,
            OdrSensor::Gyro => dmp::odr::GYRO,
            OdrSensor::Cpass => dmp::odr::CPASS,
            OdrSensor::Als => dmp::odr::ALS,
            OdrSensor::Quat6 => dmp::odr::QUAT6,
            OdrSensor::Quat9 => dmp::odr::QUAT9,
            OdrSensor::Pquat6 => dmp::odr::PQUAT6,
            OdrSensor::Geomag => dmp::odr::GEOMAG,
            OdrSensor::Pressure => dmp::odr::PRESSURE,
            OdrSensor::GyroCalibr => dmp::odr::GYRO_CALIBR,
            OdrSensor::CpassCalibr => dmp::odr::CPASS_CALIBR,
        };

        let odr_count_addr: u16 = match sensor {
            OdrSensor::Accel => dmp::odr_counter::ACCEL,
            OdrSensor::Gyro => dmp::odr_counter::GYRO,
            OdrSensor::Cpass => dmp::odr_counter::CPASS,
            OdrSensor::Als => dmp::odr_counter::ALS,
            OdrSensor::Quat6 => dmp::odr_counter::QUAT6,
            OdrSensor::Quat9 => dmp::odr_counter::QUAT9,
            OdrSensor::Pquat6 => dmp::odr_counter::PQUAT6,
            OdrSensor::Geomag => dmp::odr_counter::GEOMAG,
            OdrSensor::Pressure => dmp::odr_counter::PRESSURE,
            OdrSensor::GyroCalibr => dmp::odr_counter::GYRO_CALIBR,
            OdrSensor::CpassCalibr => dmp::odr_counter::CPASS_CALIBR,
        };

        self.device.set_sleep(false)?; // Desactivar el modo de suspensión
        self.device.low_power(false)?; // Desactivar el modo de bajo consumo

        // Write divider to the appropriate register
        let bytes = divider.to_be_bytes();
        self.device.write_mems(odr_addr, &bytes)?;
        let odr_count_zero: u16 = 0x00;
        self.device.write_mems(odr_count_addr, &odr_count_zero.to_be_bytes())?;

        self.device.low_power(true)?; // Activar el modo de bajo consumo
        
        Ok(())
    }
    
    /// Set batch mode parameters
    pub fn set_batchmode_params(&mut self, thld: u32, mask: i16) -> Result<(), Icm20948Error> {
        // Reset batch counter
        let zeros = [0u8; 4];
        self.device.write_mems(dmp::bm_batch::CNTR, &zeros)?;
        
        // Set batch threshold
        let thld_bytes = thld.to_be_bytes();
        self.device.write_mems(dmp::bm_batch::THLD, &thld_bytes)?;
        
        // Set batch mask
        let mask_bytes = mask.to_be_bytes();
        self.device.write_mems(dmp::bm_batch::MASK, &mask_bytes)?;
        
        Ok(())
    }
    
    /// Set accelerometer bias
    pub fn set_bias_acc<'a>(&mut self, bias: &'a [i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::bias::ACCEL_X, &x_bytes)?;
        self.device.write_mems(dmp::bias::ACCEL_Y, &y_bytes)?;
        self.device.write_mems(dmp::bias::ACCEL_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Set gyroscope bias
    pub fn set_bias_gyr(&mut self, bias: &[i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::bias::GYRO_X, &x_bytes)?;
        self.device.write_mems(dmp::bias::GYRO_Y, &y_bytes)?;
        self.device.write_mems(dmp::bias::GYRO_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Set compass bias
    pub fn set_bias_cmp(&mut self, bias: &[i32; 3]) -> Result<(), Icm20948Error> {
        let x_bytes = bias[0].to_be_bytes();
        let y_bytes = bias[1].to_be_bytes();
        let z_bytes = bias[2].to_be_bytes();
        
        self.device.write_mems(dmp::bias::CPASS_X, &x_bytes)?;
        self.device.write_mems(dmp::bias::CPASS_Y, &y_bytes)?;
        self.device.write_mems(dmp::bias::CPASS_Z, &z_bytes)?;
        
        Ok(())
    }
    
    /// Get accelerometer bias
    pub fn get_bias_acc(&mut self) -> Result<[i32; 3], Icm20948Error> {
        let mut x_bytes = [0u8; 4];
        let mut y_bytes = [0u8; 4];
        let mut z_bytes = [0u8; 4];
        
        self.device.read_mems(dmp::bias::ACCEL_X, &mut x_bytes)?;
        self.device.read_mems(dmp::bias::ACCEL_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::bias::ACCEL_Z, &mut z_bytes)?;
        
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
        
        self.device.read_mems(dmp::bias::GYRO_X, &mut x_bytes)?;
        self.device.read_mems(dmp::bias::GYRO_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::bias::GYRO_Z, &mut z_bytes)?;
        
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
        
        self.device.read_mems(dmp::bias::CPASS_X, &mut x_bytes)?;
        self.device.read_mems(dmp::bias::CPASS_Y, &mut y_bytes)?;
        self.device.read_mems(dmp::bias::CPASS_Z, &mut z_bytes)?;
        
        let x = i32::from_be_bytes(x_bytes);
        let y = i32::from_be_bytes(y_bytes);
        let z = i32::from_be_bytes(z_bytes);
        
        Ok([x, y, z])
    }
    
    /// Set gyro scale factor
    pub fn set_gyro_sf(&mut self, gyro_sf: i32) -> Result<(), Icm20948Error> {
        let bytes = gyro_sf.to_be_bytes();
        self.device.write_mems(dmp::fsr::GYRO, &bytes)
    }
    
    /// Sets the gyro scale factor for DMP
    pub fn dmp_set_gyro_sf(&mut self, div: u8, _gyro_level: i16) -> Result<(), Icm20948Error> {
        let gyro_level = 4i16; // Due to the addition of API dmp_icm20648_set_gyro_fsr()

        let pll: u8 = self.device.read_mems_reg::<bank1::Bank>(bank1::TIMEBASE_CORRECTION_PLL)?;
        
        let magic_constant = 264446880937391u64;
        let magic_constant_scale = 100000u64;
        let result_pll: u64;

        result_pll = if (pll & 0x80) != 0 {
            magic_constant * (1u64 << gyro_level) * (1 + div as u64) /
                (1270 - (pll & 0x7F) as u64) / magic_constant_scale

        } else {
            magic_constant * (1u64 << gyro_level) * (1 + div as u64) /
                (1270 + pll as u64) / magic_constant_scale
        };

        let gyro_sf = if result_pll > 0x7FFFFFFF {
            0x7FFFFFFF
        } else {
            result_pll as i32
        };

        // Set DMP gyro SF
        let bytes = gyro_sf.to_be_bytes();
        self.device.write_mems(dmp::scale::GYRO_SF, &bytes)
    }

    /// Set accel feedback gain
    pub fn set_accel_feedback_gain(&mut self, accel_gain: i32) -> Result<(), Icm20948Error> {
        let bytes = accel_gain.to_be_bytes();
        self.device.write_mems(dmp::scale::ACCEL_ONLY_GAIN, &bytes)
    }
    
    /// Set accel calibration parameters
    pub fn set_accel_cal_params(&mut self, accel_cal: &[i32; 3]) -> Result<(), Icm20948Error> {
        let alpha_var_bytes = accel_cal[0].to_be_bytes();
        let a_var_bytes = accel_cal[1].to_be_bytes();
        let div_bytes = accel_cal[2].to_be_bytes();
        
        self.device.write_mems(dmp::scale::ACCEL_ALPHA_VAR, &alpha_var_bytes)?;
        self.device.write_mems(dmp::scale::ACCEL_A_VAR, &a_var_bytes)?;
        
        // Only write lower 16 bits of div
        let div_short_bytes = [(div_bytes[2]), (div_bytes[3])];
        self.device.write_mems(dmp::scale::ACCEL_CAL_RATE, &div_short_bytes)?;
        
        Ok(())
    }
    
    /// Set compass cal parameters
    pub fn set_compass_cal_params(&mut self, time_buffer: u16, radius_thresh: Option<u32>) -> Result<(), Icm20948Error> {
        let time_buffer_bytes = time_buffer.to_be_bytes();        
        self.device.write_mems(dmp::compass_cal::TIME_BUFFER, &time_buffer_bytes)?;

        if radius_thresh.is_some() {
            let radius_thresh_bytes = radius_thresh.map_or([0u8; 4], |r| r.to_be_bytes());
            self.device.write_mems(dmp::compass_cal::RADIUS_3D_THRESH_ANOMALY, &radius_thresh_bytes)?;
        }
        
        Ok(())
    }
    
    /// Set compass orientation matrix
    pub fn set_compass_matrix(&mut self, compass_mtx: &[u32; 9]) -> Result<(), Icm20948Error> {
        let mtx_addrs = [
            dmp::cpass::MTX_00, dmp::cpass::MTX_01, dmp::cpass::MTX_02,
            dmp::cpass::MTX_10, dmp::cpass::MTX_11, dmp::cpass::MTX_12,
            dmp::cpass::MTX_20, dmp::cpass::MTX_21, dmp::cpass::MTX_22,
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
        self.device.read_mems(dmp::pedometer::STEPCTR, &mut bytes)?;
        
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
        self.device.write_mems(dmp::wom::ENABLE, &bytes)
    }
    
    /// Set wake on motion threshold
    pub fn set_wom_motion_threshold(&mut self, threshold: i32) -> Result<(), Icm20948Error> {
        let bytes = threshold.to_be_bytes();
        self.device.write_mems(dmp::wom::THRESHOLD, &bytes)
    }
    
    /// Set wake on motion time threshold
    pub fn set_wom_time_threshold(&mut self, threshold: u16) -> Result<(), Icm20948Error> {
        let bytes = threshold.to_be_bytes();
        self.device.write_mems(dmp::wom::CNTR_TH, &bytes)
    }
    
    /// Set gyro full-scale range
    pub fn set_gyro_fsr(&mut self, gyro_fsr_dps: u16) -> Result<(), Icm20948Error> {
    let scale = match gyro_fsr_dps {
        250 => 33554432i32,   // 2^25
        500 => 67108864i32,   // 2^26
        1000 => 134217728i32, // 2^27
        2000 => 268435456i32, // 2^28
        _ => return Err(Icm20948Error::InvalidParameter),
    };
    
    let bytes = scale.to_be_bytes();
    self.device.write_mems(dmp::fsr::GYRO, &bytes)?;
    
    // Actualizar también la escala en el dispositivo principal
    let gyro_scale = match gyro_fsr_dps {
        250 => GyroFullScale::Fs250Dps,
        500 => GyroFullScale::Fs500Dps,
        1000 => GyroFullScale::Fs1000Dps,
        2000 => GyroFullScale::Fs2000Dps,
        _ => return Err(Icm20948Error::InvalidParameter),
    };
    
    self.device.set_gyro_fullscale(gyro_scale)
}
    
    /// Set accelerometer full-scale range
    pub fn set_accel_fsr(&mut self, accel_fsr_g: u16) -> Result<(), Icm20948Error> {
        // Primer valor siempre fijo para 4g (0x04000000)
        let acc_scale = 0x04000000i32;
        let acc_scale_bytes = acc_scale.to_be_bytes();
        self.device.write_mems(dmp::fsr::ACC, &acc_scale_bytes)?;
        
        // Segundo valor ajustado según la escala seleccionada
        let acc_scale2 = match accel_fsr_g {
            2 => 0x00020000i32,
            4 => 0x00040000i32,
            8 => 0x00080000i32,
            16 => 0x00100000i32,
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        let acc_scale2_bytes = acc_scale2.to_be_bytes();
        self.device.write_mems(dmp::fsr::ACC2, &acc_scale2_bytes)?; // Asegúrate de que ACC2 es la constante correcta
        
        // Actualizar también la escala en el dispositivo principal
        let accel_scale = match accel_fsr_g {
            2 => AccelFullScale::Fs2G,
            4 => AccelFullScale::Fs4G,
            8 => AccelFullScale::Fs8G,
            16 => AccelFullScale::Fs16G,
            _ => return Err(Icm20948Error::InvalidParameter),
        };
        
        self.device.set_accel_fullscale(accel_scale)
    }
    
    /// Set EIS authentication input
    pub fn set_eis_auth_input(&mut self, eis_auth_input: i32) -> Result<(), Icm20948Error> {
        let bytes = eis_auth_input.to_be_bytes();
        self.device.write_mems(dmp::eis_auth::INPUT, &bytes)
    }
    
    /// Get EIS authentication output
    pub fn get_eis_auth_output(&mut self) -> Result<i32, Icm20948Error> {
        let mut bytes = [0u8; 4];
        self.device.read_mems(dmp::eis_auth::OUTPUT, &mut bytes)?;
        
        Ok(i32::from_be_bytes(bytes))
    }
    
    /// Set BAC rate
    pub fn set_bac_rate(&mut self, bac_odr: dmp::AlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match bac_odr {
            dmp::AlgoFreq::Freq56Hz => 0i16,
            dmp::AlgoFreq::Freq112Hz => 1i16,
            dmp::AlgoFreq::Freq225Hz => 3i16,
            dmp::AlgoFreq::Freq450Hz => 7i16,
            dmp::AlgoFreq::Freq900Hz => 15i16,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::bac::BAC_RATE, &bytes)
    }
    
    /// Set B2S rate
    pub fn set_b2s_rate(&mut self, accel_odr: dmp::AlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match accel_odr {
            dmp::AlgoFreq::Freq56Hz => 0i16,
            dmp::AlgoFreq::Freq112Hz => 1i16,
            dmp::AlgoFreq::Freq225Hz => 3i16,
            dmp::AlgoFreq::Freq450Hz => 7i16,
            dmp::AlgoFreq::Freq900Hz => 15i16,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::bac::B2S_RATE, &bytes)
    }
    
    /// Set B2S orientation matrix
    pub fn set_b2s_matrix(&mut self, b2s_mtx: &[u32; 9]) -> Result<(), Icm20948Error> {
        let mtx_addrs = [
            dmp::b2s_mtx::B2S_MTX_00, dmp::b2s_mtx::B2S_MTX_01, dmp::b2s_mtx::B2S_MTX_02,
            dmp::b2s_mtx::B2S_MTX_10, dmp::b2s_mtx::B2S_MTX_11, dmp::b2s_mtx::B2S_MTX_12,
            dmp::b2s_mtx::B2S_MTX_20, dmp::b2s_mtx::B2S_MTX_21, dmp::b2s_mtx::B2S_MTX_22,
        ];
        
        for i in 0..9 {
            let bytes = b2s_mtx[i].to_be_bytes();
            self.device.write_mems(mtx_addrs[i], &bytes)?;
        }
        
        Ok(())
    }
    
    /// Set Flip-Pickup rate
    pub fn set_fp_rate(&mut self, accel_odr: dmp::AlgoFreq) -> Result<(), Icm20948Error> {
        let odr = match accel_odr {
            dmp::AlgoFreq::Freq56Hz => 0i32,
            dmp::AlgoFreq::Freq112Hz => 1i32,
            dmp::AlgoFreq::Freq225Hz => 3i32,
            dmp::AlgoFreq::Freq450Hz => 7i32,
            dmp::AlgoFreq::Freq900Hz => 15i32,
        };
        
        let bytes = odr.to_be_bytes();
        self.device.write_mems(dmp::flip_pickup::RATE, &bytes)
    }
    
    /// Reset BAC states
    pub fn reset_bac_states(&mut self) -> Result<(), Icm20948Error> {
        let zero_i32 = [0u8; 4];
        let zero_i16 = [0u8; 2];
        
        // Reset all BAC state variables
        let i32_addrs = [
            dmp::bac_state::STATE, dmp::bac_state::STATE_PREV, 
            dmp::bac_state::ACT_ON, dmp::bac_state::ACT_OFF,
            dmp::bac_state::STILL_S_F, dmp::bac_state::RUN_S_F,
            dmp::bac_state::DRIVE_S_F, dmp::bac_state::WALK_S_F,
            dmp::bac_state::SMD_S_F, dmp::bac_state::BIKE_S_F,
            dmp::bac_state::E1_SHORT, dmp::bac_state::E2_SHORT,
            dmp::bac_state::E3_SHORT, dmp::bac_state::VAR_RUN,
            dmp::bac_state::DRIVE_CONFIDENCE, dmp::bac_state::WALK_CONFIDENCE,
            dmp::bac_state::SMD_CONFIDENCE, dmp::bac_state::BIKE_CONFIDENCE,
            dmp::bac_state::STILL_CONFIDENCE, dmp::bac_state::RUN_CONFIDENCE,
            dmp::bac_state::MODE_CNTR, dmp::bac_state::STATE_T_PREV,
            dmp::bac_state::ACT_T_ON, dmp::bac_state::ACT_T_OFF,
            dmp::bac_state::STATE_WRDBS_PREV, dmp::bac_state::ACT_WRDBS_ON,
            dmp::bac_state::ACT_WRDBS_OFF
        ];
        
        for addr in i32_addrs.iter() {
            self.device.write_mems(*addr, &zero_i32)?;
        }
        
        let i16_addrs = [
            dmp::bac_state::ACT_ON_OFF, dmp::bac_state::PREV_ACT_ON_OFF,
            dmp::bac_state::CNTR
        ];
        
        for addr in i16_addrs.iter() {
            self.device.write_mems(*addr, &zero_i16)?;
        }
        
        Ok(())
    }
    
    /// Set pedometer y-ratio
    pub fn set_ped_y_ratio(&mut self, ped_y_ratio: i32) -> Result<(), Icm20948Error> {
        let bytes = ped_y_ratio.to_be_bytes();
        self.device.write_mems(dmp::pedometer::PED_Y_RATIO, &bytes)
    }
    
    /// Set orientation parameters
    pub fn set_orientation_params(&mut self, orientation_params: &[i32; 4]) -> Result<(), Icm20948Error> {
        let q0_bytes = orientation_params[0].to_be_bytes();
        let q1_bytes = orientation_params[1].to_be_bytes();
        let q2_bytes = orientation_params[2].to_be_bytes();
        let q3_bytes = orientation_params[3].to_be_bytes();
        
        self.device.write_mems(dmp::orientation::Q0_QUAT6, &q0_bytes)?;
        self.device.write_mems(dmp::orientation::Q1_QUAT6, &q1_bytes)?;
        self.device.write_mems(dmp::orientation::Q2_QUAT6, &q2_bytes)?;
        self.device.write_mems(dmp::orientation::Q3_QUAT6, &q3_bytes)?;
        
        Ok(())
    }

    /// Convierte los datos raw del acelerómetro a g
    pub fn convert_accel_raw_to_g(&self, raw_data: &[i16; 3]) -> [f32; 3] {
        let factor = match self.device.base_state.accel_fullscale {
            AccelFullScale::Fs2G => 2.0 / 32768.0,
            AccelFullScale::Fs4G => 4.0 / 32768.0,
            AccelFullScale::Fs8G => 8.0 / 32768.0,
            AccelFullScale::Fs16G => 16.0 / 32768.0,
        };
        let mut g_data = [0.0; 3];
        for i in 0..3 {
            g_data[i] = (raw_data[i] as f32) * factor;
        }
        g_data
    }
    
    /// Convierte los datos raw del giroscopio a dps
    pub fn convert_gyro_raw_to_dps(&self, raw_data: &[i16; 3]) -> [f32; 3] {
        let factor = match self.device.base_state.gyro_fullscale {
            GyroFullScale::Fs250Dps => 250.0 / 32768.0,
            GyroFullScale::Fs500Dps => 500.0 / 32768.0,
            GyroFullScale::Fs1000Dps => 1000.0 / 32768.0,
            GyroFullScale::Fs2000Dps => 2000.0 / 32768.0,
        };
        let mut dps_data = [0.0; 3];
        for i in 0..3 {
            dps_data[i] = (raw_data[i] as f32) * factor;
        }
        dps_data        
    }
    
    pub fn sensor_type_2_android_sensor(sensor: Sensor) -> AndroidSensor  {
        match sensor {
            Sensor::Accelerometer => AndroidSensor::Accelerometer,
            Sensor::Gyroscope => AndroidSensor::Gyroscope,
            Sensor::RawAccelerometer => AndroidSensor::RawAccelerometer,
            Sensor::RawGyroscope => AndroidSensor::RawGyroscope,
            Sensor::MagneticFieldUncalibrated => AndroidSensor::MagneticFieldUncalibrated,
            Sensor::GyroscopeUncalibrated => AndroidSensor::GyroscopeUncalibrated,
            Sensor::ActivityClassification => AndroidSensor::ActivityClassification,
            Sensor::StepDetector => AndroidSensor::StepDetector,
            Sensor::StepCounter => AndroidSensor::StepCounter,
            Sensor::GameRotationVector => AndroidSensor::GameRotationVector,
            Sensor::RotationVector => AndroidSensor::RotationVector,
            Sensor::GeomagneticRotationVector => AndroidSensor::GeomagneticRotationVector,
            Sensor::GeomagneticField => AndroidSensor::GeomagneticField,
            Sensor::WakeupSignificantMotion => AndroidSensor::WakeupSignificantMotion,
            Sensor::FlipPickup => AndroidSensor::FlipPickup,
            Sensor::WakeupTiltDetector => AndroidSensor::WakeupTiltDetector,
            Sensor::Gravity => AndroidSensor::Gravity,
            Sensor::LinearAcceleration => AndroidSensor::LinearAcceleration,
            Sensor::Orientation => AndroidSensor::Orientation,
            Sensor::B2S => AndroidSensor::B2S,
            _ => AndroidSensor::NumMax,
        }
    }
    
    pub fn android_sensor_2_sensor_type(sensor: AndroidSensor) -> Sensor {
        match sensor {
            AndroidSensor::Accelerometer => Sensor::Accelerometer,
            AndroidSensor::Gyroscope => Sensor::Gyroscope,
            AndroidSensor::RawAccelerometer => Sensor::RawAccelerometer,
            AndroidSensor::RawGyroscope => Sensor::RawGyroscope,
            AndroidSensor::MagneticFieldUncalibrated => Sensor::MagneticFieldUncalibrated,
            AndroidSensor::GyroscopeUncalibrated => Sensor::GyroscopeUncalibrated,
            AndroidSensor::ActivityClassification => Sensor::ActivityClassification,
            AndroidSensor::StepDetector => Sensor::StepDetector,
            AndroidSensor::StepCounter => Sensor::StepCounter,
            AndroidSensor::GameRotationVector => Sensor::GameRotationVector,
            AndroidSensor::RotationVector => Sensor::RotationVector,
            AndroidSensor::GeomagneticRotationVector => Sensor::GeomagneticRotationVector,
            AndroidSensor::GeomagneticField => Sensor::GeomagneticField,
            AndroidSensor::WakeupSignificantMotion => Sensor::WakeupSignificantMotion,
            AndroidSensor::FlipPickup => Sensor::FlipPickup,
            AndroidSensor::WakeupTiltDetector => Sensor::WakeupTiltDetector,
            AndroidSensor::Gravity => Sensor::Gravity,
            AndroidSensor::LinearAcceleration => Sensor::LinearAcceleration,
            AndroidSensor::Orientation => Sensor::Orientation,
            _ => Sensor::Max,
        }
    }
}