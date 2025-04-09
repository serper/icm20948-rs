use crate::device::{Icm20948, Icm20948Error};
use crate::interface::Interface;
use crate::types;
use embedded_hal::blocking::delay::DelayMs;

/// Loads the DMP firmware from SRAM
/// 
/// # Arguments
/// * `device` - The ICM20948 device instance
/// * `firmware_data` - The firmware image data
/// * `load_addr` - Address for loading the image
/// 
/// # Returns
/// * `Result<(), Icm20948Error>` - Ok if successful, Err with error code otherwise
pub fn load_firmware<I, D, E>(
    device: &mut Icm20948<I, D>,
    firmware_data: &[u8],
    load_addr: u16,
) -> Result<(), Icm20948Error>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    // Icm20948Error: From<E>,
{
    if device.is_firmware_loaded() {
        return Ok(());
    }

    // Write DMP memory
    let mut data_index = 0;
    let mut mem_addr = load_addr;
    let mut remaining_size = firmware_data.len();

    while remaining_size > 0 {
        let write_size = remaining_size.min(types::INV_MAX_SERIAL_WRITE);
        let adjusted_write_size = if (mem_addr & 0xff) + write_size as u16 > 0x100 {
            // Moved across a bank
            ((mem_addr & 0xff) + write_size as u16 - 0x100) as usize
        } else {
            write_size
        };

        // Slice creado de forma segura con lifetimes claros
        let slice_to_write = &firmware_data[data_index..(data_index + adjusted_write_size)];
        device.write_mems(
            mem_addr, 
            slice_to_write
        )?;

        data_index += adjusted_write_size;
        remaining_size -= adjusted_write_size;
        mem_addr += adjusted_write_size as u16;
    }

    // Verify DMP memory
    let mut data_index = 0;
    let mut mem_addr = load_addr;
    let mut remaining_size = firmware_data.len();
    let mut data_cmp = [0u8; types::INV_MAX_SERIAL_READ];

    while remaining_size > 0 {
        let read_size = remaining_size.min(types::INV_MAX_SERIAL_READ);
        let adjusted_read_size = if (mem_addr & 0xff) + read_size as u16 > 0x100 {
            // Moved across a bank
            ((mem_addr & 0xff) + read_size as u16 - 0x100) as usize
        } else {
            read_size
        };

        device.read_mems(mem_addr, &mut data_cmp[0..adjusted_read_size])?;

        // Compare data
        for i in 0..adjusted_read_size {
            if data_cmp[i] != firmware_data[data_index + i] {
                return Err(Icm20948Error::FirmwareVerificationFailed);
            }
        }

        data_index += adjusted_read_size;
        remaining_size -= adjusted_read_size;
        mem_addr += adjusted_read_size as u16;
    }

    // Mark firmware as loaded
    device.set_firmware_loaded(true);
    Ok(())
}

/// Loads the DMP3 firmware for the ICM20948
/// 
/// # Arguments
/// * `device` - The ICM20948 device instance
/// 
/// # Returns
/// * `Result<(), Icm20948Error>` - Ok if successful, Err with error code otherwise
pub fn load_dmp3_firmware<I, D, E>(
    device: &mut Icm20948<I, D>,
) -> Result<(), Icm20948Error>
where
    I: Interface<Error = E>,
    D: DelayMs<u32>,
    Icm20948Error: From<E>,
{
    // Usar directamente el firmware DMP3 del módulo
    load_firmware(device, crate::dmp3_firmware::DMP3_FIRMWARE, types::DMP_LOAD_START)
}

/// Gets the DMP start address
pub fn get_dmp_start_address() -> u16 {
    0x1000 // DMP_START_ADDRESS from C code
}
