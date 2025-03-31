//! Ejemplo avanzado para ICM20948 en Linux con DMP y sensores aumentados
//!
//! Este ejemplo muestra cómo configurar y utilizar el DMP (Digital Motion Processor)
//! del ICM20948 para obtener cuaterniones, ángulos de orientación, gravedad y 
//! aceleración lineal en Linux.
//!
//! Para ejecutar: cargo run --example linux_advanced

use icm20948_rs::{self, Icm20948Error, Icm20948};
use linux_embedded_hal::{Delay, I2cdev};
use std::time::Duration;
use std::thread;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

// Ejecuta un ejemplo de DMP continuo hasta que se presione Ctrl+C
fn run_dmp_example_continuous<E>(
    mut device: Icm20948<I2cdev, Delay>,
    running: Arc<AtomicBool>
) -> Result<(), Icm20948Error> {
    // Inicializar el dispositivo
    device.initialize()?;
    println!("Dispositivo inicializado");
    
    // Configurar y activar el DMP
    println!("Configurando DMP...");
    
    // Configurar escalas para el acelerómetro y giroscopio
    device.set_accel_fullscale(icm20948_rs::AccelFullScale::Fs2G)?;
    device.set_gyro_fullscale(icm20948_rs::GyroFullScale::Fs250Dps)?;
    
    // Configurar tasas de muestreo
    device.set_accelerometer_sample_rate(icm20948_rs::SampleRate::Custom(100))?; // 100Hz
    device.set_gyroscope_sample_rate(icm20948_rs::SampleRate::Custom(100))?; // 100Hz
    
    println!("Leyendo datos DMP. Presiona Ctrl+C para detener...");
    
    // Leer datos en bucle hasta que se presione Ctrl+C
    while running.load(Ordering::SeqCst) {
        // Leer datos de acelerómetro
        if let Ok(accel) = device.read_accel_raw() {
            println!("Aceleración: x={:.2}, y={:.2}, z={:.2}", accel[0], accel[1], accel[2]);
        }
        
        // Leer datos de giroscopio
        if let Ok(gyro) = device.read_gyroscope() {
            println!("Giroscopio: x={:.2}, y={:.2}, z={:.2}", gyro[0], gyro[1], gyro[2]);
        }
        
        // Leer datos de temperatura
        if let Ok(temp) = device.read_temp_raw() {
            println!("Temperatura: {:.2}°C", temp);
        }
        
        println!("-------------------");
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("Ejemplo DMP finalizado");
    Ok(())
}

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
    let device = icm20948_rs::new_i2c_device(i2c, 0x68, delay);
    
    // Ejecutar ejemplo DMP continuo
    match run_dmp_example_continuous(device, running) {
        Ok(_) => println!("Ejemplo DMP ejecutado correctamente"),
        Err(e) => eprintln!("Error durante la ejecución del ejemplo DMP: {:?}", e),
    }
}
