# ICM20948-RS

Biblioteca Rust para el sensor de movimiento InvenSense ICM20948, un IMU (Unidad de Medición Inercial) de 9 ejes con giroscopio, acelerómetro y magnetómetro.

## Características

- Soporte completo para las funcionalidades del ICM20948
- Integración con DMP (Digital Motion Processor)
- Soporte para magnetómetros AK (AK09916, AK09911, AK09912, AK08963)
- Lectura de acelerómetro, giroscopio y magnetómetro
- Procesamiento de sensores aumentados (cuaterniones, matriz de rotación)
- Cálculo de orientación, aceleración lineal y gravedad
- Funcionalidades avanzadas:
  - Wake-on-Motion (WoM)
  - Detección de paso (pedómetro)
  - Clasificación de actividad
  - Detección de gestos
- Compatible con el trait `embedded-hal`

## Requisitos

- Rust 1.51 o superior
- Dependencias:
  - `embedded-hal`
  - Otras dependencias según el target (I2C/SPI)

## Instalación

Añade la siguiente línea a tu Cargo.toml:

```toml
[dependencies]
icm20948-rs = { git = "https://github.com/tuusuario/icm20948-rs" }
```

## Uso básico

```rust
use icm20948_rs::{Icm20948, new_i2c_device, AccelFullScale, GyroFullScale};
use linux_embedded_hal::{I2cdev, Delay};

fn main() {
    // Crear instancia de I2C y Delay
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let delay = Delay {};
    
    // Crear dispositivo ICM20948 (dirección por defecto 0x68)
    let mut device = new_i2c_device(i2c, 0x68, delay);
    
    // Inicializar el dispositivo
    device.initialize().unwrap();
    
    // Configurar escalas
    device.set_accel_fullscale(AccelFullScale::Fs2G).unwrap();
    device.set_gyro_fullscale(GyroFullScale::Fs250Dps).unwrap();
    
    // Leer datos
    if let Ok(accel) = device.read_accelerometer() {
        println!("Aceleración: x={:.2}G, y={:.2}G, z={:.2}G", accel[0], accel[1], accel[2]);
    }
    
    if let Ok(gyro) = device.read_gyroscope() {
        println!("Giroscopio: x={:.2}°/s, y={:.2}°/s, z={:.2}°/s", gyro[0], gyro[1], gyro[2]);
    }
    
    // Activar y leer magnetómetro
    device.setup_compass(compass::CompassType::AK09916, 0x0C).unwrap();
    if let Ok(mag) = device.read_compass(100) {
        println!("Magnetómetro: x={:.2}µT, y={:.2}µT, z={:.2}µT", mag.x, mag.y, mag.z);
    }
}
```

## Ejemplos

El proyecto incluye varios ejemplos:

- `linux_basic`: Ejemplo básico para leer acelerómetro, giroscopio y magnetómetro
- `linux_advanced`: Uso avanzado con DMP (Digital Motion Processor)
- `linux_realtime_plot`: Visualización gráfica de datos del acelerómetro

Para ejecutar los ejemplos:

```bash
cargo run --example linux_basic
cargo run --example linux_advanced
cargo run --example linux_realtime_plot --features="plotting"
```

## Características del DMP

El DMP (Digital Motion Processor) integrado permite procesamiento avanzado de movimiento:

```rust
// Inicializar DMP
let mut dmp_device = icm20948_rs::dmp::DmpDriverIcm20948::new(device);
dmp_device.initialize().unwrap();

// Activar sensores
dmp_device.enable_dmp_sensor(Sensor::Accelerometer, true).unwrap();
dmp_device.enable_dmp_sensor(Sensor::Orientation, true).unwrap();

// Activar FIFO
dmp_device.dmp_fifo_enable(true).unwrap();
dmp_device.enable_dmp(true).unwrap();
```

## Estructura del proyecto

```
src/
  ├── augmented.rs    - Sensores aumentados (orientación, aceleración linear)
  ├── base.rs         - Funcionalidades base
  ├── compass.rs      - Soporte para magnetómetros
  ├── config.rs       - Configuraciones
  ├── controls.rs     - Control de sensores 
  ├── conversion.rs   - Conversión de unidades
  ├── device.rs       - Implementación principal del dispositivo
  ├── dmp.rs          - Soporte para Digital Motion Processor
  ├── dmp3_firmware.rs - Firmware para el DMP
  ├── dmp_fifo.rs     - Manejo de FIFO para DMP
  ├── fifo.rs         - Operaciones de FIFO
  ├── interface.rs    - Interfaces de comunicación
  ├── lib.rs          - Punto de entrada de la biblioteca
  ├── register/       - Definición de registros
  ├── selftest.rs     - Auto-test y calibración
  └── types.rs        - Tipos y estructuras
```

## Depuración

El proyecto incluye scripts para depuración remota:

- debug-arm.sh: Configuración para depuración en ARM
- remote_debug.sh: Depuración remota

## Contribuir

Las contribuciones son bienvenidas. Por favor, sigue estas pautas:

1. Usa 4 espacios para indentación y formato rustfmt
2. Usa PascalCase para structs/enums, y snake_case para funciones/variables
3. Agrupa importaciones: crates externos primero, luego módulos internos
4. Usa el enum `Icm20948Error` y el tipo Result personalizado para manejo de errores
5. Documenta la API pública con comentarios  y los módulos con 

## Licencia

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Referencias

- [Datasheet ICM20948](https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf)
- [Documentación oficial Rust](https://doc.rust-lang.org/book/)
- [Documentación embedded-hal](https://docs.rs/embedded-hal/)
