//! Ejemplo avanzado para ICM20948 en Linux con DMP y sensores aumentados
//!
//! Este ejemplo muestra cómo configurar y utilizar el DMP (Digital Motion Processor)
//! del ICM20948 para obtener cuaterniones, ángulos de orientación, gravedad y 
//! aceleración lineal en Linux.
//!
//! Para ejecutar: cargo run --example linux_advanced

use icm20948_rs::{self, Icm20948Error, Icm20948, dmp, firmware::load_dmp3_firmware};
use linux_embedded_hal::{Delay, I2cdev};
use std::time::Duration;
use std::thread;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

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
    
    // Inicializar el dispositivo
    if let Err(e) = device.initialize() {
        eprintln!("Error al inicializar el dispositivo: {:?}", e);
        return;
    }
    println!("Dispositivo inicializado correctamente");

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

    // Configurar y activar el DMP
    let dmp_device = icm20948_rs::dmp::DmpDriverIcm20948::new(device);
    if let Err(e) = load_dmp3_firmware(&dmp_device) {
        eprintln!("Error al cargar el firmware DMP3: {:?}", e);
        return;
    }
    println!("Firmware DMP3 cargado correctamente");

    /// Implementar ejemplo de uso del DMP

}
