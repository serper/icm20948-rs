// ... existing code ...

/// Set Gyro Scale Factor (GYRO_SF) with PLL timebase correction
/// Critical for correct gyro integration (Yaw stability)
///
/// Matches inv_icm20948_set_gyro_sf logic from InvenSense driver
pub fn dmp_set_gyro_sf(&mut self, div: u8, _gyro_level: i32) -> Result<(), Icm20948Error> {
    // NOTE: InvenSense driver forces gyro_level = 4 (225Hz base) for this calc, regardless of FSR
    let gyro_level = 4;

    // Read TIMEBASE_CORRECTION_PLL (Bank 0, Reg 0x28)
    let pll = self.device.read_reg(bank0::TIMEBASE_CORRECTION_PLL)?;

    // Calculate gyro_sf
    let magic_constant = 264446880937391u64;
    let magic_constant_scale = 100000u64;

    let pll_val = (pll & 0x7F) as u64;
    let term = if (pll & 0x80) != 0 {
        1270 - pll_val
    } else {
        1270 + pll_val
    };

    // Formula: (Magic * 2^gyro_level * (1 + div)) / term / MagicScale
    // gyro_level=4 -> 1<<4 = 16
    let result_ll = magic_constant
        .checked_mul(1u64 << gyro_level)
        .unwrap_or(0)
        .checked_mul((1 + div) as u64)
        .unwrap_or(0)
        .checked_div(term)
        .unwrap_or(0)
        .checked_div(magic_constant_scale)
        .unwrap_or(0);

    let gyro_sf = if result_ll > 0x7FFFFFFF {
        0x7FFFFFFFi32
    } else {
        result_ll as i32
    };

    let bytes = gyro_sf.to_be_bytes();
    self.device.write_mems(dmp::GYRO_SF, &bytes)
}
