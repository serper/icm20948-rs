//! Ejemplo de visualización en tiempo real de datos del ICM20948 en Linux
//!
//! Este ejemplo muestra cómo visualizar gráficamente los datos del sensor
//! en tiempo real usando la biblioteca plotters.
//!
//! Para ejecutar: cargo run --example linux_realtime_plot
//!
//! Dependencias necesarias:
//! - plotters = "0.3"
//! - crossterm = "0.25"

use icm20948_rs::{self, Icm20948};
use linux_embedded_hal::{Delay, I2cdev};
use plotters::prelude::*;
use std::error::Error;
use std::time::{Duration, Instant};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

const PLOT_WIDTH: u32 = 800;
const PLOT_HEIGHT: u32 = 600;
const PLOT_POINTS: usize = 100;

fn main() -> Result<(), Box<dyn Error>> {
    println!("ICM20948 - Ejemplo de gráfico en tiempo real");
    
    // Flag para controlar la ejecución del programa
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    
    // Configurar el manejador para Ctrl+C
    ctrlc::set_handler(move || {
        println!("\nDeteniendo el programa...");
        r.store(false, Ordering::SeqCst);
    }).expect("Error al configurar el manejador de Ctrl+C");
    
    // Crear instancia de I2C para Linux
    let i2c = I2cdev::new("/dev/i2c-0")?;
    let delay = Delay {};
    
    // Crear dispositivo ICM20948
    let mut device = icm20948_rs::new_i2c_device(i2c, 0x68, delay);
    
    // Inicializar el dispositivo
    device.initialize()?;
    println!("Dispositivo inicializado correctamente");
    
    // Configuraciones iniciales
    device.set_accel_fullscale(icm20948_rs::AccelFullScale::Fs2G)?;
    
    // Vectores para almacenar datos históricos
    let mut accel_x = vec![0.0; PLOT_POINTS];
    let mut accel_y = vec![0.0; PLOT_POINTS];
    let mut accel_z = vec![0.0; PLOT_POINTS];
    
    // Crear la ventana de gráfico
    let root = BitMapBackend::new("accel_plot.png", (PLOT_WIDTH, PLOT_HEIGHT))
        .into_drawing_area();
    
    println!("Graficando datos del acelerómetro. Presiona Ctrl+C para detener...");
    
    // Instante para controlar la frecuencia de refresco
    let mut last_update = Instant::now();
    
    // Loop hasta que se presione Ctrl+C
    while running.load(Ordering::SeqCst) {
        // Control de frecuencia de refresco (10 veces por segundo)
        if last_update.elapsed() < Duration::from_millis(100) {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        last_update = Instant::now();
        
        // Leer datos del acelerómetro convertidos a G
        if let Ok(accel_data) = device.read_accelerometer() {
            // Desplazar los datos antiguos
            accel_x.remove(0);
            accel_y.remove(0);
            accel_z.remove(0);
            
            // Añadir nuevos datos
            accel_x.push(accel_data[0] as f64);
            accel_y.push(accel_data[1] as f64);
            accel_z.push(accel_data[2] as f64);
            
            // Limpiar el área de dibujo
            root.fill(&WHITE)?;
            
            // Crear el contexto del gráfico
            let mut chart = ChartBuilder::on(&root)
                .caption("Datos de aceleración", ("sans-serif", 30))
                .margin(10)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .build_cartesian_2d(0..PLOT_POINTS, -2.0..2.0)?;
            
            // Configurar el gráfico
            chart.configure_mesh()
                .x_labels(5)
                .y_labels(5)
                .x_desc("Muestras")
                .y_desc("Aceleración (G)")
                .draw()?;
            
            // Dibujar las líneas
            chart.draw_series(LineSeries::new(
                (0..PLOT_POINTS).map(|i| (i, accel_x[i])),
                &RED,
            ))?
            .label("X")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
            
            chart.draw_series(LineSeries::new(
                (0..PLOT_POINTS).map(|i| (i, accel_y[i])),
                &GREEN,
            ))?
            .label("Y")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));
            
            chart.draw_series(LineSeries::new(
                (0..PLOT_POINTS).map(|i| (i, accel_z[i])),
                &BLUE,
            ))?
            .label("Z")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));
            
            // Mostrar leyenda
            chart.configure_series_labels()
                .background_style(&WHITE.mix(0.8))
                .border_style(&BLACK)
                .draw()?;
            
            // Presentar el gráfico
            root.present()?;
        }
    }
    
    println!("Gráfico guardado como 'accel_plot.png'");
    println!("Ejemplo finalizado");
    
    Ok(())
}
