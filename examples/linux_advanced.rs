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
use crate::icm20948_rs::dmp_fifo::DmpFifoError;
use std::time::Duration;
use std::thread;
use std::io::{Write, Read};
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::sync::mpsc;

#[derive(Debug)]
struct FooterInfo {
    raw: u16,
    high_flags: u8,  // bits 12–15
    mid_flags: u8,   // bits 4–11
    low_flags: u8,   // bits 0–3
}

impl FooterInfo {
    fn from_raw(raw: u16) -> Self {
        let high_flags = ((raw >> 12) & 0x0F) as u8;
        let mid_flags = ((raw >> 4) & 0xFF) as u8;
        let low_flags = (raw & 0x0F) as u8;
        FooterInfo {
            raw,
            high_flags,
            mid_flags,
            low_flags,
        }
    }

    fn describe(&self) {
        println!("Footer: 0x{:04X}", self.raw);
        println!("  High flags (15-12):  {:04b} → {}", self.high_flags, match self.high_flags {
            0x7 => "Fusión completa (Gyro+Accel+Mag)",
            0x4 => "Fusión parcial (Gyro+Accel)",
            0x3 => "Solo Gyro o fallback",
            _ => "Desconocido o reservado",
        });

        println!("  Mid flags (11-4):     {:08b}", self.mid_flags);
        println!("  Low flags (3-0):      {:04b} → {}", self.low_flags, match self.low_flags {
            0x2 => "Código especial / quizás 'usable'",
            0x0 => "Sin precisión reportada",
            _ => "Desconocido",
        });
    }

    fn fusion_status_description(high: u8, low: u8) -> &'static str {
        match (high, low) {
            (0x7, 0x2) => "✅ Fusión completa (9DOF)",
            (0x4, 0x0..=0x4) => "🟡 Fusión sin magnetómetro",
            (0x1, _) => "🔴 Sin fusión o inicializando",
            (0x2, _) => "🟠 Parcial o incierto",
            _ => "❓ Estado desconocido",
        }
    }
}

fn main() {
    println!("ICM20948 - Ejemplo avanzado (DMP)");

    // Flag para controlar la ejecución del programa
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Recurso que necesitarás liberar al terminar
    let dmp_device_for_cleanup = Arc::new(Mutex::new(None::<icm20948_rs::dmp::DmpDriverIcm20948<I2cdev, Delay>>));
    let dmp_device_cleanup_ref = dmp_device_for_cleanup.clone();

    let cleanup = move || {
        if let Ok(mut device) = dmp_device_cleanup_ref.lock() {
            if let Some(ref mut dmp_device) = *device {
                // Guardar en un fichero los datos de calibración
                let mut calibration_data = [0; 36]; // 9 * 4 bytes
                calibration_data[0..3].copy_from_slice(&dmp_device.get_bias_acc().unwrap_or_else(|_| [0; 3]));
                calibration_data[3..6].copy_from_slice(&dmp_device.get_bias_gyr().unwrap_or_else(|_| [0; 3]));
                calibration_data[6..9].copy_from_slice(&dmp_device.get_bias_cmp().unwrap_or_else(|_| [0; 3]));

                // Guardar en un fichero los datos de calibración
                if let Err(e) = std::fs::File::create("mpucal.dat").and_then(|mut file| {
                    let calibration_data_bytes = bytemuck::cast_slice(&calibration_data);
                    file.write_all(calibration_data_bytes)
                }) {
                    eprintln!("Error al guardar datos de calibración: {:?}", e);
                } else {
                    println!("Datos de calibración guardados en mpucal.dat");
                    print!("Datos de calibración: ");
                    for i in 0..9 {
                        print!("{:?} ", calibration_data[i]);
                    }
                    println!();
                }

                // Deshabilitar FIFO y DMP antes de liberar el recurso
                dmp_device.dmp_fifo_enable(false).unwrap_or_else(|_| eprintln!("Error al deshabilitar FIFO"));
                dmp_device.enable_dmp(false).unwrap_or_else(|_| eprintln!("Error al deshabilitar DMP"));
            }
        }
    };

    // Configurar el manejador para Ctrl+C
    ctrlc::set_handler(move || {
        println!("\nDeteniendo el programa...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error al configurar el manejador de Ctrl+C");

    #[cfg(unix)]
    {
        use std::os::unix::net::UnixDatagram;
        use signal_hook::{consts::signal::*, iterator::Signals};
        
        // Crear un socket para recibir señales
        let socket_path = format!("/tmp/icm20948-signal-{}", std::process::id());
        let socket = UnixDatagram::bind(&socket_path).expect("No se pudo crear socket para señales");
        let socket_clone = socket.try_clone().expect("No se pudo clonar socket");
        
        let r2 = running.clone();
        
        // Registrar handlers para múltiples señales
        let mut signals = Signals::new(&[
            SIGHUP, SIGINT, SIGTERM, SIGQUIT, SIGABRT,
        ]).expect("Error al registrar manejadores de señales");
        
        std::thread::spawn(move || {
            for sig in signals.forever() {
                println!("\nRecibida señal: {}", sig);
                r2.store(false, Ordering::SeqCst);
                let _ = socket_clone.send(&[1]);  // Notificar al bucle principal
                break;
            }
        });
    }

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

    // let orientation_params: [i32; 4] = [
    //     0,                // w = 0
    //     -1073741824,                // x = 0
    //     0,                // y = 1.0 (Q30)
    //     -1073741824                 // z = 0
    // ];

    // dmp_device.set_orientation_params(&orientation_params).unwrap_or_else(|e| {
    //     eprintln!("Error al establecer parámetros de orientación: {:?}", e);
    // });

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
    
    // Cargar datos de calibración si existen en un fichero: mpucal.dat
    // Si existe cargar los datos de calibración consistente en 9 datos de 32 bits (i32)
    // Abrir fichero mpucal.dat y leer los datos
    let mut calibration_data = [0; 9];
    if let Ok(mut file) = std::fs::File::open("mpucal.dat") {
        if let Ok(size) = file.read(&mut calibration_data) {
            if size == 36 { // 9 * 4 bytes
                let bias_acc = bytemuck::cast_slice::<u8, i32>(&calibration_data[0..12]);
                let bias_gyr = bytemuck::cast_slice::<u8, i32>(&calibration_data[12..24]);
                let bias_cmp = bytemuck::cast_slice::<u8, i32>(&calibration_data[24..36]);

                dmp_device.set_bias_acc(&bias_acc.try_into().unwrap()).unwrap_or_else(|_| eprintln!("Error al establecer el bias de aceleración"));
                dmp_device.set_bias_gyr(&bias_gyr.try_into().unwrap()).unwrap_or_else(|_| eprintln!("Error al establecer el bias de giroscopio"));
                dmp_device.set_bias_cmp(&bias_cmp.try_into().unwrap()).unwrap_or_else(|_| eprintln!("Error al establecer el bias de magnetómetro"));
                println!("Datos de calibración cargados correctamente desde mpucal.dat");
            } else {
                eprintln!("Error: Tamaño de datos de calibración incorrecto: {}", size);
            }
        } else {
            eprintln!("Error al leer el fichero de calibración");
        }
    } else {
        eprintln!("No se encontró el fichero de calibración, usando valores por defecto");
    }   

    dmp_device.dmp_fifo_enable(true).unwrap_or_else(|_| eprintln!("Error al habilitar FIFO"));
    dmp_device.enable_dmp(true).unwrap_or_else(|_| eprintln!("Error al habilitar DMP"));
    dmp_device.set_fifo_watermark(1024).unwrap_or_else(|_| eprintln!("Error al configurar el watermark del FIFO"));
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
        let mut last_successful_read = std::time::Instant::now();
        let read_timeout = Duration::from_millis(500); // Timeout para evitar bloqueo en lectura

        while thread_running.load(Ordering::SeqCst) {
            // Usar un scope para limitar el tiempo que se mantiene el mutex bloqueado
            let read_result = {
                match thread_dmp_device.try_lock() {
                    Ok(mut device) => {
                        // Intentar leer con un timeout para evitar bloqueos
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            device.read_dmp_fifo_data(&mut dmp_state)
                        })) {
                            Ok(result) => result,
                            Err(_) => {
                                eprintln!("Error de pánico en lectura DMP");
                                Err(DmpFifoError::DataError)
                            }
                        }
                    },
                    Err(_) => {
                        // No se pudo adquirir el mutex, probablemente esté en uso
                        thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                }
            };

            // Comprobar resultado de lectura
            match read_result {
                Ok(_) => {
                    // Enviar datos solo si la lectura fue exitosa
                    if let Err(_) = tx.send(dmp_state.clone()) {
                        eprintln!("Error al enviar datos del DMP - canal cerrado?");
                    }
                    last_successful_read = std::time::Instant::now();
                },
                Err(e) => {
                    // Si pasó mucho tiempo desde la última lectura exitosa, reiniciar el dispositivo
                    if last_successful_read.elapsed() > read_timeout {
                        eprintln!("Error en lectura DMP: {:?}, intentando resetear", e);
                        if let Ok(mut device) = thread_dmp_device.try_lock() {
                            let _ = device.reset_fifo();
                            // let _ = device.reset_dmp();
                            dmp_state.last_header = 0xFFFF;
                            dmp_state.last_header2 = 0xFFFF;
                        }
                        last_successful_read = std::time::Instant::now();
                    }
                }
            }
            
            // Esperar un tiempo corto antes de la siguiente lectura
            thread::sleep(Duration::from_millis(10));
        }
    });

    // Procesar datos recibidos en el canal
    let mut last_display_time = std::time::Instant::now();
    let display_interval = Duration::from_millis(200);
    let mut last_data_time = std::time::Instant::now();
    let data_timeout = Duration::from_secs(3); // Tiempo para considerar que hay un problema de datos

    while running.load(Ordering::SeqCst) {
        // Intentar recibir datos con timeout corto para responder rápido a señales de parada
        match rx.recv_timeout(Duration::from_millis(50)) {
            Ok(dmp_state) => {
                last_data_time = std::time::Instant::now();
                
                // Mostrar datos solo si es el momento
                if last_display_time.elapsed() >= display_interval {
                    // Procesamiento de datos (tu código actual)
                    if let Some(accel) = dmp_state.accel_data.as_ref() {
                        // Usar try_lock para evitar bloqueos
                        if let Ok(device) = dmp_device.try_lock() {
                            let accel_g = device.convert_accel_raw_to_g(accel);
                            println!("Accel: x: {:?}, y: {:?}, z: {:?}", accel_g[0], accel_g[1], accel_g[2]);
                        }
                    }
                    if let Some(gyro) = dmp_state.gyro_data.as_ref() {
                        if let Ok(device) = dmp_device.try_lock() {
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
                    print!("Header: {:X?} Header2: {:X?} Size: {:?} Count: {:?} Footer: {:016b} Accuracy Accel: {:?}, Gyro: {:?}, Compass: {:?}\n",
                        dmp_state.header, dmp_state.header2, dmp_state.packet_size, dmp_state.count, dmp_state.footer, dmp_state.accel_accuracy, dmp_state.gyro_accuracy, dmp_state.compass_accuracy);
                    
                    let footer = FooterInfo::from_raw(dmp_state.footer);
                    println!("Footer: 0x{:04X} → {}", dmp_state.footer, FooterInfo::fusion_status_description(footer.high_flags, footer.low_flags));
                    // footer.describe();
                    println!();
                    print!("Packet: {:X?}\n", dmp_state.packet);
                    last_display_time = std::time::Instant::now();
                }
            },
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // No se recibieron datos pero el canal sigue abierto
                if last_data_time.elapsed() > data_timeout {
                    eprintln!("Advertencia: No se han recibido datos en más de 3 segundos");
                    last_data_time = std::time::Instant::now(); // Resetear para no spamear
                }
            },
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                // El canal se ha cerrado, probablemente el thread terminó
                eprintln!("Error: El thread de lectura ha finalizado inesperadamente");
                break;
            }
        }
        
        // Pequeña pausa para no consumir CPU innecesariamente
        thread::sleep(Duration::from_millis(1));
    }

    println!("Programa detenido. Limpiando recursos...");
    cleanup(); // Llamar a la función de limpieza
    // Esperar a que el thread termine
    dmp_thread.join().unwrap_or_else(|_| eprintln!("Error al unir el thread del DMP"));

    #[cfg(unix)]
    {
        let _ = std::fs::remove_file(format!("/tmp/icm20948-signal-{}", std::process::id()));
    }
}
