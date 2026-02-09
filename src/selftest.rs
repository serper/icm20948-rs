//! Módulo para realizar el auto-test del ICM20948
//!
//! Este módulo proporciona funciones para ejecutar el auto-test del sensor,
//! verificando la correcta funcionalidad del acelerómetro y giroscopio.

use crate::controls;
use crate::device::{Icm20948, Icm20948Error};
use crate::interface::Interface;
use crate::register::registers::bank0;
use crate::register::registers::bank1;
use crate::register::registers::bank2;
use crate::types;
use embedded_hal::delay::DelayNs;
use log;

/// Estructura para almacenar los resultados del auto-test
#[derive(Debug, Default, Clone, Copy)]
pub struct SelfTestResult {
    pub accel_x: i8,
    pub accel_y: i8,
    pub accel_z: i8,
    pub gyro_x: i8,
    pub gyro_y: i8,
    pub gyro_z: i8,
}

/// Ejecuta el auto-test del ICM20948
///
/// Esta función realiza una serie de lecturas y escrituras en registros específicos
/// para activar el auto-test y verificar los resultados.
pub fn run_self_test<I, D, E>(device: &mut Icm20948<I, D>) -> Result<(bool, bool), Icm20948Error>
where
    I: Interface<Error = E>,
    D: DelayNs,
{
    // Guardar la configuración actual del sensor
    let orig_pwr_mgmt_1 = device.read_reg::<bank0::Bank>(bank0::PWR_MGMT_1)?;
    let orig_user_ctrl = device.read_reg::<bank0::Bank>(bank0::USER_CTRL)?;
    let orig_accel_config_1 = device.read_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1)?;
    let orig_accel_config_2 = device.read_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_2)?;
    let orig_gyro_config_1 = device.read_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1)?;
    let orig_gyro_config_2 = device.read_reg::<bank2::Bank>(bank2::GYRO_CONFIG_2)?;

    // Resetear el dispositivo
    device.soft_reset()?;

    // Wake up el dispositivo
    device.wakeup()?;
    device.set_low_power_mode(true)?;

    // Configurar el acelerómetro y giroscopio para el auto-test
    device.set_accel_fullscale(types::AccelFullScale::Fs2G)?;
    device.set_accel_lpf(controls::AccelLpfSetting::Lp246_0Hz)?;
    device.set_accel_divider(10)?;
    device.write_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_2, 0x02)?;

    device.set_gyro_fullscale(types::GyroFullScale::Fs250Dps)?;
    device.set_gyro_lpf(controls::GyroLpfSetting::Lp196_6Hz)?;
    device.set_gyro_divider(10)?;
    device.write_reg::<bank2::Bank>(bank2::GYRO_CONFIG_2, 0x03)?;

    // Activar el auto-test seteando los bits 4:2 de ACCEL_CONFIG_2 y GYRO_CONFIG_2
    device.modify_reg::<bank2::Bank, _>(bank2::ACCEL_CONFIG_2, |val| val | 0x1C)?;
    device.modify_reg::<bank2::Bank, _>(bank2::GYRO_CONFIG_2, |val| val | 0x1C)?;
    device.delay.delay_ms(200);

    // Leer los resultados del auto-test
    let st_result = SelfTestResult {
        accel_x: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_X_ACCEL)? as i8,
        accel_y: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_Y_ACCEL)? as i8,
        accel_z: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_Z_ACCEL)? as i8,
        gyro_x: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_X_GYRO)? as i8,
        gyro_y: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_Y_GYRO)? as i8,
        gyro_z: device.read_reg::<bank1::Bank>(bank1::SELF_TEST_Z_GYRO)? as i8,
    };

    // Verificar los resultados del auto-test
    let accel_ok = st_result.accel_x != 0 && st_result.accel_y != 0 && st_result.accel_z != 0;
    let gyro_ok = st_result.gyro_x != 0 && st_result.gyro_y != 0 && st_result.gyro_z != 0;

    // 8. Restaurar la configuración original del sensor
    device.write_reg::<bank0::Bank>(bank0::PWR_MGMT_1, orig_pwr_mgmt_1)?;
    device.write_reg::<bank0::Bank>(bank0::USER_CTRL, orig_user_ctrl)?;
    device.write_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1, orig_accel_config_1)?;
    device.write_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_2, orig_accel_config_2)?;
    device.write_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1, orig_gyro_config_1)?;
    device.write_reg::<bank2::Bank>(bank2::GYRO_CONFIG_2, orig_gyro_config_2)?;

    Ok((accel_ok, gyro_ok))
}

/// Calibra el acelerómetro y giroscopio
pub fn calibrate<I, D, E>(
    device: &mut Icm20948<I, D>,
    must_apply: bool,
) -> Result<(), Icm20948Error>
where
    I: Interface<Error = E>,
    D: DelayNs,
{
    // Wake up el dispositivo
    device.wakeup()?;
    device.set_low_power_mode(true)?;

    // Guardar offsets HW actuales y deshabilitar compensación durante el muestreo
    let prev_accel_offsets = device.get_accel_hw_offsets()?;
    let _prev_gyro_offsets = device.get_gyro_hw_offsets()?;
    if must_apply {
        device.set_accel_hw_offsets([0, 0, 0])?;
        device.set_gyro_hw_offsets([0, 0, 0])?;
        device.modify_reg::<bank2::Bank, _>(bank2::ACCEL_CONFIG_2, |val| val & !0x01)?;
        device.modify_reg::<bank2::Bank, _>(bank2::GYRO_CONFIG_2, |val| val & !0x01)?;
    }

    // Configurar el acelerómetro y giroscopio para el auto-test
    let last_accel_fs = device.get_accel_fullscale()?;
    device.set_accel_fullscale(types::AccelFullScale::Fs2G)?;
    device.set_accel_lpf(controls::AccelLpfSetting::Lp246_0Hz)?;
    device.set_accel_divider(10)?;
    device.write_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_2, 0x02)?;

    let last_gyro_fs = device.get_gyro_fullscale()?;
    device.set_gyro_fullscale(types::GyroFullScale::Fs250Dps)?;
    device.set_gyro_lpf(controls::GyroLpfSetting::Lp196_6Hz)?;
    device.set_gyro_divider(10)?;
    device.write_reg::<bank2::Bank>(bank2::GYRO_CONFIG_2, 0x03)?;

    // Realizar la calibración del acelerómetro y giroscopio
    const NUM_SAMPLES: usize = 100;

    // Arrays para almacenar las sumas acumuladas
    let mut accel_sum = [0i32; 3];
    let mut gyro_sum = [0i32; 3];

    // Esperar a que el sensor se estabilice
    device.delay.delay_ms(100);

    // Tomar varias muestras y acumularlas
    for _ in 0..NUM_SAMPLES {
        // Leer valores brutos del hardware
        let accel_raw = device.accel_read_hw_reg_data()?;
        let gyro_raw = device.gyro_read_hw_reg_data()?;

        // Acumular valores para cada eje
        // Acumular valores para cada eje
        for (i, val) in accel_sum.iter_mut().enumerate() {
            *val += accel_raw[i] as i32;
        }
        for (i, val) in gyro_sum.iter_mut().enumerate() {
            *val += gyro_raw[i] as i32;
        }

        // Pequeña pausa entre muestras
        device.delay.delay_ms(5);
    }

    // Calcular promedios (valores brutos del sensor)
    let accel_avg = [
        (accel_sum[0] / NUM_SAMPLES as i32) as i16,
        (accel_sum[1] / NUM_SAMPLES as i32) as i16,
        (accel_sum[2] / NUM_SAMPLES as i32) as i16,
    ];

    let gyro_avg = [
        (gyro_sum[0] / NUM_SAMPLES as i32) as i16,
        (gyro_sum[1] / NUM_SAMPLES as i32) as i16,
        (gyro_sum[2] / NUM_SAMPLES as i32) as i16,
    ];

    // Calcular offsets (negativo del promedio para compensar)
    // Para el acelerómetro: compensar X e Y a 0, Z al valor de 1g según la escala
    let accel_fullscale = device.get_accel_fullscale()?;
    let gravity_value = match accel_fullscale {
        types::AccelFullScale::Fs2G => 16384, // Valor típico para 1g en escala ±2g
        types::AccelFullScale::Fs4G => 8192,  // Valor típico para 1g en escala ±4g
        types::AccelFullScale::Fs8G => 4096,  // Valor típico para 1g en escala ±8g
        types::AccelFullScale::Fs16G => 2048, // Valor típico para 1g en escala ±16g
    };

    // Detectar qué eje está alineado con la gravedad
    let axis_magnitudes = [accel_avg[0].abs(), accel_avg[1].abs(), accel_avg[2].abs()];
    let accel_avg_g = [
        accel_avg[0] as f32 / gravity_value as f32,
        accel_avg[1] as f32 / gravity_value as f32,
        accel_avg[2] as f32 / gravity_value as f32,
    ];
    let abs_g = [
        accel_avg_g[0].abs(),
        accel_avg_g[1].abs(),
        accel_avg_g[2].abs(),
    ];

    // Encontrar el eje con la mayor magnitud (probablemente el afectado por gravedad)
    let (gravity_axis, _) = axis_magnitudes
        .iter()
        .enumerate()
        .max_by_key(|&(_, &val)| val)
        .unwrap_or((2, &axis_magnitudes[2]));

    // Determinar dirección (positiva o negativa)
    let gravity_sign = if accel_avg[gravity_axis] > 0 { 1 } else { -1 };

    // Validar que el eje dominante esté suficientemente alineado con la gravedad.
    // Si no está alineado, evitar calibrar el acelerómetro para no introducir sesgo.
    let mut abs_sorted = abs_g;
    abs_sorted.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
    let dominant = abs_sorted[0];
    let secondary = abs_sorted[1];
    let apply_accel_cal = dominant >= 0.7 && secondary <= 0.2;
    if !apply_accel_cal {
        log::warn!(
            "Accel calibration skipped: gravity not aligned (dominant={:.3}g, secondary={:.3}g)",
            dominant,
            secondary
        );
    }

    // Para el giroscopio: compensar todos los ejes a 0
    // Para el giroscopio: compensar todos los ejes a 0
    let mut gyro_offsets = [0i16; 3];
    for (i, offset) in gyro_offsets.iter_mut().enumerate() {
        // Cambiar a complemento a 2 y convertir de 250dps a 1000dps (>>2)
        *offset = -(gyro_avg[i] >> 2);
    }

    // Para el acelerómetro: leer valores actuales y aplicar delta
    // Convertir raw a offset regs (2048 LSB/g) de forma independiente al fullscale.
    let gravity_lsb = gravity_value as f32;
    let offset_scale = 2048.0 / gravity_lsb;
    let mut accel_offsets = [0i16; 3];
    let mut reg_addr = bank1::XA_OFFS_H;

    for i in 0..3 {
        // Leer el offset actual (16 bits en dos registros)
        let h_byte = device.read_reg::<bank1::Bank>(reg_addr)?;
        let l_byte = device.read_reg::<bank1::Bank>(reg_addr + 1)?;
        let old_offset = ((h_byte as i16) << 8) | (l_byte as i16);

        // Calcular delta en LSB de offset (2048 LSB/g)
        let mut delta_offset = (accel_avg[i] as f32 * offset_scale).round() as i16;

        if i == gravity_axis {
            // Aplicar compensación de gravedad
            delta_offset -= (gravity_sign * 2048) as i16;
        }
        // Calcular nuevo offset (restar delta del valor actual)
        accel_offsets[i] = old_offset - delta_offset;

        reg_addr += 3; // Avanzar al siguiente conjunto de registros (saltando el no utilizado)
    }

    // Escribir offsets en registros correspondientes
    // El ICM20948 almacena los offsets en registros específicos
    // Banco 1 contiene los registros de offset
    if must_apply {
        if apply_accel_cal {
            device.set_accel_hw_offsets(accel_offsets)?;
            device.modify_reg::<bank2::Bank, _>(bank2::ACCEL_CONFIG_2, |val| val | 0x01)?;
        } else {
            device.set_accel_hw_offsets(prev_accel_offsets)?;
        }

        device.set_gyro_hw_offsets(gyro_offsets)?;
        device.modify_reg::<bank2::Bank, _>(bank2::GYRO_CONFIG_2, |val| val | 0x01)?;
    }

    // Guardar los offsets en la estructura del dispositivo para futuras referencias
    log::info!("Calibración completada");
    log::info!(
        "Offsets de acelerómetro: [{}, {}, {}]",
        accel_offsets[0], accel_offsets[1], accel_offsets[2]
    );
    log::info!(
        "Offsets de giroscopio: [{}, {}, {}]",
        gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]
    );

    // Restaurar Accel y Gyro Full Scale
    device.set_accel_fullscale(last_accel_fs)?;
    device.set_gyro_fullscale(last_gyro_fs)?;

    // Esperar un poco antes de continuar
    device.delay.delay_ms(100);

    Ok(())
}
