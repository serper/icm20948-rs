
// Example Rhai script for ICM20948

print("Initializing ICM20948...");
icm.initialize();

print("Enabling DMP...");
icm.enable_dmp(true);

print("Enabling acceleration and gyro sensors...");
icm.enable_sensor("accel", true);
icm.enable_sensor("gyro", true);
icm.enable_sensor("quat9", true); // Orientation

print("Setting sensor rates...");
icm.set_sensor_rate("accel", 10); // Div 10
icm.set_sensor_rate("gyro", 10);

print("Calibrating (simulated)...");
// icm.calibrate_gyro();

print("Reading data loop...");

for i in 0..10 {
    let data = icm.read_data();
    
    if data.contains("accel") {
        print(`Accel: ${data.accel}`);
    }
    
    if data.contains("quat") {
        print(`Quat: ${data.quat}`);
    }
    
    // Simulate delay
    // sleep(0.1); // Built-in sleep might not cover all cases, but for demo we just loop
}

print("Done.");
