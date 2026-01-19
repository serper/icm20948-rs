use icm20948_rs::api_rhai::{register_icm_api, RhaiIcm20948};
use icm20948_rs::dmp::DmpDriverIcm20948;
use icm20948_rs::new_i2c_device;
use linux_embedded_hal::{Delay, I2cdev};
use rhai::Engine;
use std::path::Path;
use std::sync::{Arc, Mutex};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting Rhai Runner...");

    // 1. Setup hardware
    let i2c_bus = I2cdev::new("/dev/i2c-1")?;
    // Address 0x69 (or 0x68)
    let icm = new_i2c_device(i2c_bus, 0x69, Delay);

    // 2. Initialize Rhai engine
    let mut engine = Engine::new();

    // 3. Register ICM API
    // Create DmpDriver from device
    let dmp_driver = DmpDriverIcm20948::new(icm);

    // Wrap in Arc<Mutex<>>
    let shared_driver = Arc::new(Mutex::new(dmp_driver));

    // Create Rhai wrapper
    let rhai_icm = RhaiIcm20948::new(shared_driver);

    register_icm_api(&mut engine);

    // 4. Create scope and add the device
    let mut scope = rhai::Scope::new();
    scope.push("icm", rhai_icm);

    // 5. Run script
    let script_path = Path::new("examples/script.rhai");
    if script_path.exists() {
        println!("Running script: {:?}", script_path);
        engine.run_file_with_scope(&mut scope, script_path.into())?;
    } else {
        println!("Script not found at {:?}", script_path);
    }

    Ok(())
}
