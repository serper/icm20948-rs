//! Ejemplo avanzado para ICM20948 en Linux con DMP y sensores aumentados
//!
//! Este ejemplo muestra cómo configurar y utilizar el DMP (Digital Motion Processor)
//! del ICM20948 para obtener cuaterniones, ángulos de orientación, gravedad y 
//! aceleración lineal en Linux.
//!
//! Para ejecutar: cargo run --example linux_advanced

use icm20948_rs::{self, dmp_fifo};
use icm20948_rs::types::{Sensor, OdrSensor};
use linux_embedded_hal::{Delay, I2cdev};
use std::time::Duration;
use std::thread;
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::sync::mpsc;

fn main() {
    println!("ICM20948 - Ejemplo avanzado (DMP)");

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

    // Configurar y activar el DMP
    let mut dmp_device = icm20948_rs::dmp::DmpDriverIcm20948::new(device);
    if let Err(e) = dmp_device.initialize() {
        eprintln!("Error al inicializar el DMP: {:?}", e);
        return;
    }

    let orientation_params: [i32; 4] = [
        0,                // w = 0
        -1073741824,                // x = 0
        0,                // y = 1.0 (Q30)
        -1073741824                 // z = 0
    ];

    dmp_device.set_orientation_params(&orientation_params).unwrap_or_else(|e| {
        eprintln!("Error al establecer parámetros de orientación: {:?}", e);
    });

    // Configurar sensores y FIFO
    dmp_device.enable_dmp_sensor(Sensor::RawAccelerometer, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor Accel"));
    // dmp_device.enable_dmp_sensor(Sensor::Gyroscope, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor Gyro"));
    // dmp_device.enable_dmp_sensor(Sensor::MagneticFieldUncalibrated, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor MagneticFieldUncalibrated"));
    dmp_device.enable_dmp_sensor(Sensor::Orientation, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor GeomagneticRotationVector"));
    // dmp_device.enable_dmp_sensor(Sensor::Gravity, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor Gravity"));
    // dmp_device.enable_dmp_sensor(Sensor::StepDetector, true).unwrap_or_else(|_| eprintln!("Error al habilitar el sensor StepDetector"));
    
    // Configurar la frecuencia de muestreo
    dmp_device.set_sensor_rate(OdrSensor::Accel, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor Accel"));
    // dmp_device.set_sensor_rate(OdrSensor::Gyro, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor Gyro"));
    dmp_device.set_sensor_rate(OdrSensor::Quat9, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor Quat9"));
    // dmp_device.set_sensor_rate(OdrSensor::GyroCalibr, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor GyroCalibr"));
    // dmp_device.set_sensor_rate(OdrSensor::CpassCalibr, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor CpassCalibr"));
    // dmp_device.set_sensor_rate(OdrSensor::Cpass, 10).unwrap_or_else(|_| eprintln!("Error al configurar la frecuencia de muestreo del sensor Cpass"));
    
    dmp_device.dmp_fifo_enable(true).unwrap_or_else(|_| eprintln!("Error al habilitar FIFO"));
    dmp_device.enable_dmp(true).unwrap_or_else(|_| eprintln!("Error al habilitar DMP"));
    dmp_device.set_fifo_watermark(128).unwrap_or_else(|_| eprintln!("Error al configurar el watermark del FIFO"));
    dmp_device.reset_dmp().unwrap_or_else(|_| eprintln!("Error al resetear el DMP"));
    dmp_device.reset_fifo().unwrap_or_else(|_| eprintln!("Error al resetear el FIFO"));

    // Envolver el dispositivo DMP en un Arc<Mutex<>> para compartirlo entre threads
    let dmp_device = Arc::new(Mutex::new(dmp_device));

    // Canal para comunicar el thread de lectura con el principal
    let (tx, rx) = mpsc::channel();

    // Clonar las referencias para el thread
    let thread_running = running.clone();
    let thread_dmp_device = dmp_device.clone();

    // Crear thread para leer datos del DMP
    let dmp_thread = std::thread::spawn(move || {
        let mut dmp_state = dmp_fifo::DmpFifoState::default();
        while thread_running.load(Ordering::SeqCst) {
            // Bloquear el mutex solo durante la lectura
            if let Ok(mut device) = thread_dmp_device.lock() {
                if device.read_dmp_fifo_data(&mut dmp_state).is_ok() {
                    tx.send(dmp_state.clone()).unwrap_or_else(|_| eprintln!("Error al enviar datos del DMP"));
                }
            }
            thread::sleep(Duration::from_millis(10)); // Ajustar según la frecuencia de interrupción
        }
    });

    // Configurar interrupción en PE10
    // Aquí se debería configurar el pin PE10 como entrada y manejar la interrupción
    // Esto depende de la biblioteca GPIO que se utilice en el sistema Linux

    // Procesar datos recibidos en el canal
    let mut last_display_time = std::time::Instant::now();
    let display_interval = Duration::from_millis(200);

    while running.load(Ordering::SeqCst) {
        if let Ok(dmp_state) = rx.recv_timeout(Duration::from_millis(100)) {
            if last_display_time.elapsed() >= display_interval {
                if let Some(accel) = dmp_state.accel_data.as_ref() {
                    // Bloquear el mutex solo durante la conversión
                    if let Ok(device) = dmp_device.lock() {
                        let accel_g = device.convert_accel_raw_to_g(accel);
                        println!("Accel: x: {:?}, y: {:?}, z: {:?}", accel_g[0], accel_g[1], accel_g[2]);
                    }
                }
                if let Some(gyro) = dmp_state.gyro_data.as_ref() {
                    if let Ok(device) = dmp_device.lock() {
                        let gyro_dps = device.convert_gyro_raw_to_dps(gyro);
                        println!("Gyro: x: {:?}, y: {:?}, z: {:?}", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
                    }
                }
                if let Some(gyro) = dmp_state.gyro_calibr.as_ref() {
                    println!("Gyro Calib: x: {:?}, y: {:?}, z: {:?}", gyro.x, gyro.y, gyro.z);
                }
                if let Some(compass) = dmp_state.compass_data.as_ref() {
                    println!("Compass: x: {:?}, y: {:?}, z: {:?}", compass[0], compass[1], compass[2]);
                }
                if let Some(compass) = dmp_state.compass_calibr.as_ref() {
                    println!("Compass Calibr: w: {:?}, x: {:?}, y: {:?}, z: {:?}", compass.w, compass.x, compass.y, compass.z);
                    let euler_angles = icm20948_rs::dmp_fifo::quaternion_to_euler(compass);
                    let mut roll = euler_angles[0] + 180.0;
                    let pitch = -euler_angles[1];
                    let yaw = (-euler_angles[2] + 360.0) % 360.0;
                    if roll > 180.0 {
                        roll -= 360.0;
                    }
                    println!("Compass Angles: roll: {:?}, pitch: {:?}, yaw: {:?}", roll, pitch, yaw);
                }
                if let Some(quaternion) = dmp_state.quaternion9.as_ref() {
                    println!("Quaternion9: w: {:?}, x: {:?}, y: {:?}, z: {:?}", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
                    let euler_angles = icm20948_rs::dmp_fifo::quaternion_to_euler(quaternion);
                    let mut roll = euler_angles[0] + 180.0;
                    let pitch = -euler_angles[1];
                    let yaw = (-euler_angles[2] + 360.0) % 360.0;
                    if roll > 180.0 {
                        roll -= 360.0;
                    }
                    println!("Euler Angles: roll: {:?}, pitch: {:?}, yaw: {:?}", roll, pitch, yaw);
                }
                if let Some(quaternion) = dmp_state.geomag_data.as_ref() {
                    println!("Geomag: w: {:?}, x: {:?}, y: {:?}, z: {:?}", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
                    let euler_angles = icm20948_rs::dmp_fifo::quaternion_to_euler(quaternion);
                    let mut roll = euler_angles[0] + 180.0;
                    let pitch = -euler_angles[1];
                    let yaw = (-euler_angles[2] + 360.0) % 360.0;
                    if roll > 180.0 {
                        roll -= 360.0;
                    }
                    println!("Euler Angles: roll: {:?}, pitch: {:?}, yaw: {:?}", roll, pitch, yaw);
                }
                print!("Header: {:X?} Header2: {:X?} Size: {:?} Count: Footer: {:?} Accuracy Accel: {:?}, Gyro: {:?}, Compass: {:?}\n",
                    dmp_state.header, dmp_state.header2, dmp_state.packet_size, dmp_state.footer, dmp_state.accel_accuracy, dmp_state.gyro_accuracy, dmp_state.compass_accuracy);
                last_display_time = std::time::Instant::now();
            }
        }
    }

    // Esperar a que el thread termine
    dmp_thread.join().unwrap_or_else(|_| eprintln!("Error al unir el thread del DMP"));
}
