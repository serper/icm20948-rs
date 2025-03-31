use icm20948_rs::{self, AccelFullScale, GyroFullScale, compass, selftest, controls, register::registers::bank0, register::registers::bank2, device::CHIP_LP_ENABLE};
use linux_embedded_hal::{Delay, I2cdev};
use std::thread;
use std::time::Duration;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

fn main() {
    println!("ICM20948 - Ejemplo básico");
    
    // Flag para controlar la ejecución del programa
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    
    // Configurar el manejador para Ctrl+C
    ctrlc::set_handler(move || {
        println!("\nDeteniendo el programa...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error al configurar el manejador de Ctrl+C");
    
    // Crear instancia de I2C para Linux
    let i2c = match I2cdev::new("/dev/i2c-0") {
        Ok(i2c) => i2c,
        Err(e) => {
            eprintln!("Error al abrir dispositivo I2C: {:?}", e);
            return;
        }
    };
    let delay = Delay {};
    
    // Crear dispositivo ICM20948 con la dirección I2C estándar
    let mut device = icm20948_rs::new_i2c_device(i2c, 0x68, delay);
    
    // Inicializar el dispositivo
    if let Err(e) = device.initialize() {
        eprintln!("Error al inicializar el dispositivo: {:?}", e);
        return;
    }
    println!("Dispositivo inicializado correctamente");

    // Ejecutamos el self-test del dispositivo
    match selftest::run_self_test(&mut device) {
        Ok((accel_pass, gyro_pass)) => {
            if accel_pass && gyro_pass {
                println!("Self-test exitoso para acelerómetro y giroscopio");
            } else {
                println!("Self-test fallido para {}{}",
                    if !accel_pass { "acelerómetro " } else { "" },
                    if !gyro_pass { "giroscopio" } else { "" }
                );
            }
        },
        Err(e) => {
            eprintln!("Error en el self-test: {:?}", e);
            return;
        }
    }
    
    // Activar magnetómetro
    if let Err(e) = device.setup_compass(compass::CompassType::AK09916, 0x0C) {
        eprintln!("Error al activar el magnetómetro: {:?}", e);
    } else {
        if device.compass_is_connected() {
            println!("Magnetómetro conectado correctamente");
        } else {
            eprintln!("Error: Magnetómetro no conectado");
        }
    }

    // Configurar tasas de muestreo
    if let Err(e) =  device.set_accelerometer_sample_rate(controls::SampleRate::Custom(100)) { // 100Hz
        eprintln!("Error al configurar la tasa de muestreo del acelerómetro: {:?}", e);
    }
    if let Err(e) = device.set_gyroscope_sample_rate(controls::SampleRate::Custom(100)) { // 100Hz
        eprintln!("Error al configurar la tasa de muestreo del giroscopio: {:?}", e);
    }
    
    // Configurar LPF (Low Pass Filter) para el acelerómetro y giroscopio
    if let Err(e) = device.set_accel_lpf(controls::AccelLpfSetting::Lp246_0Hz) {
        eprintln!("Error al configurar el LPF del acelerómetro: {:?}", e);
    }
    if let Err(e) = device.set_gyro_lpf(controls::GyroLpfSetting::Lp196_6Hz) {
        eprintln!("Error al configurar el LPF del giroscopio: {:?}", e);
    }

    // Configurar escalas
    if let Err(e) = device.set_accel_fullscale(AccelFullScale::Fs4G) {
        eprintln!("Error al configurar la escala del acelerómetro: {:?}", e);
    }
    
    if let Err(e) = device.set_gyro_fullscale(GyroFullScale::Fs250Dps) {
        eprintln!("Error al configurar la escala del giroscopio: {:?}", e);
    }

    device.set_chip_power_state(CHIP_LP_ENABLE, 0);
    
    // Calibración de sensores
    if let Err(e) = selftest::calibrate(&mut device, true) {
        eprintln!("Error al calibrar sensores: {:?}", e);
        return;
    }
    println!("Sensores calibrados correctamente");

    // Configurar el magnetómetro
    if let Err(e) = device.set_compass_mode(compass::compass_rd::MODE_CONT_100HZ) {
        eprintln!("Error al configurar el modo del magnetómetro: {:?}", e);
    }
    // // Compass self-test
    // if let Err(e) = device.check_compass_self_test() {
    //     eprintln!("Error en el self-test del magnetómetro: {:?}", e);
    // } else {
    //     println!("Self-test del magnetómetro exitoso");
    // }

    // // Guardar la configuración actual del sensor
    // let orig_pwr_mgmt_1 = match device.read_reg::<bank0::Bank>(bank0::PWR_MGMT_1) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer PWR_MGMT_1: {:?}", e);
    //         return;
    //     }
    // };
    // let orig_user_ctrl = match device.read_reg::<bank0::Bank>(bank0::USER_CTRL) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer USER_CTRL: {:?}", e);
    //         return;
    //     }
    // };
    // let orig_accel_config_1 = match device.read_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_1) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer ACCEL_CONFIG_1: {:?}", e);
    //         return;
    //     }
    // };
    // let orig_accel_config_2 = match device.read_reg::<bank2::Bank>(bank2::ACCEL_CONFIG_2) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer ACCEL_CONFIG_2: {:?}", e);
    //         return;
    //     }
    // };
    // let orig_gyro_config_1 = match device.read_reg::<bank2::Bank>(bank2::GYRO_CONFIG_1) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer GYRO_CONFIG_1: {:?}", e);
    //         return;
    //     }
    // };
    // let orig_gyro_config_2 = match device.read_reg::<bank2::Bank>(bank2::GYRO_CONFIG_2) {
    //     Ok(value) => value,
    //     Err(e) => {
    //         eprintln!("Error al leer GYRO_CONFIG_2: {:?}", e);
    //         return;
    //     }
    // };

    // // Imprimir la configuración original en binario
    // println!("Configuración: PWR_MGMT_1: {:08b}, USER_CTRL: {:08b}, ACCEL_CONFIG_1: {:08b}, ACCEL_CONFIG_2: {:08b}, GYRO_CONFIG_1: {:08b}, GYRO_CONFIG_2: {:08b}", 
    //     orig_pwr_mgmt_1, orig_user_ctrl, orig_accel_config_1, orig_accel_config_2, orig_gyro_config_1, orig_gyro_config_2);

    // Leer datos continuamente hasta que se presione Ctrl+C
    println!("Leyendo datos. Presiona Ctrl+C para detener...");
    
    while running.load(Ordering::SeqCst) {
        // Leer y mostrar datos del acelerómetro en G
        match device.read_accelerometer() {
            Ok(accel) => println!("Aceleración: x={:.2}G, y={:.2}G, z={:.2}G", accel[0], accel[1], accel[2]),
            Err(e) => eprintln!("Error al leer acelerómetro: {:?}", e),
        }
        
        // Leer y mostrar datos del giroscopio en grados/segundo
        match device.read_gyroscope() {
            Ok(gyro) => println!("Giroscopio: x={:.2}°/s, y={:.2}°/s, z={:.2}°/s", gyro[0], gyro[1], gyro[2]),
            Err(e) => eprintln!("Error al leer giroscopio: {:?}", e),
        }
        
        // Leer y mostrar datos del magnetómetro en microteslas
        match device.read_compass(100) {
            Ok(mag) => println!("Magnetómetro: x={:.2}µT, y={:.2}µT, z={:.2}µT", mag.x, mag.y, mag.z),
            Err(e) => eprintln!("Error al leer magnetómetro: {:?}", e),
        }

        // Leer y mostrar temperatura en Celsius
        match device.read_temperature() {
            Ok(temp) => println!("Temperatura: {:.2}°C", temp),
            Err(e) => eprintln!("Error al leer temperatura: {:?}", e),
        }
        println!("-------------------");
        thread::sleep(Duration::from_millis(200));
    }
    
    println!("Ejemplo finalizado");
}
