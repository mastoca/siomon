use std::fs::{File, OpenOptions};
use std::os::unix::io::AsRawFd;

// Linux I2C ioctl request codes (from <linux/i2c-dev.h>)
const I2C_SLAVE: libc::c_ulong = 0x0703;
const I2C_SLAVE_FORCE: libc::c_ulong = 0x0706;
const I2C_SMBUS: libc::c_ulong = 0x0720;

// SMBus transfer direction
const I2C_SMBUS_READ: u8 = 1;
const I2C_SMBUS_WRITE: u8 = 0;

// SMBus transaction sizes
const I2C_SMBUS_BYTE_DATA: u32 = 2;
const I2C_SMBUS_WORD_DATA: u32 = 3;

/// Argument structure for the I2C_SMBUS ioctl.
#[repr(C)]
struct I2cSmbusIoctlData {
    read_write: u8,
    command: u8,
    size: u32,
    data: *mut I2cSmbusData,
}

/// Union matching the kernel's `union i2c_smbus_data`.
#[repr(C)]
union I2cSmbusData {
    byte: u8,
    word: u16,
    block: [u8; 34],
}

/// An open handle to one I2C slave device for SMBus register reads.
pub struct SmbusDevice {
    file: File,
}

impl SmbusDevice {
    /// Open `/dev/i2c-{bus}` and bind to the given 7-bit slave address.
    ///
    /// Tries `I2C_SLAVE` first; falls back to `I2C_SLAVE_FORCE` if the
    /// address is already claimed by a kernel driver (EBUSY).
    pub fn open(bus: u32, addr: u16) -> std::io::Result<Self> {
        let path = format!("/dev/i2c-{bus}");
        let file = OpenOptions::new().read(true).write(true).open(&path)?;

        let ret = unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE, addr as libc::c_int) };
        if ret < 0 {
            let err = std::io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EBUSY) {
                // Address claimed by kernel driver — force access
                let ret2 =
                    unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE_FORCE, addr as libc::c_int) };
                if ret2 < 0 {
                    return Err(std::io::Error::last_os_error());
                }
            } else {
                return Err(err);
            }
        }

        Ok(Self { file })
    }

    /// Read a single byte from `register` via SMBus byte-data protocol.
    pub fn read_byte_data(&self, register: u8) -> std::io::Result<u8> {
        let mut data = I2cSmbusData { byte: 0 };
        let mut args = I2cSmbusIoctlData {
            read_write: I2C_SMBUS_READ,
            command: register,
            size: I2C_SMBUS_BYTE_DATA,
            data: &mut data,
        };

        let ret = unsafe { libc::ioctl(self.file.as_raw_fd(), I2C_SMBUS, &mut args as *mut _) };
        if ret < 0 {
            return Err(std::io::Error::last_os_error());
        }

        Ok(unsafe { data.byte })
    }

    /// Write a single byte to `register` via SMBus byte-data protocol.
    pub fn write_byte_data(&self, register: u8, value: u8) -> std::io::Result<()> {
        let mut data = I2cSmbusData { byte: value };
        let mut args = I2cSmbusIoctlData {
            read_write: I2C_SMBUS_WRITE,
            command: register,
            size: I2C_SMBUS_BYTE_DATA,
            data: &mut data,
        };

        let ret = unsafe { libc::ioctl(self.file.as_raw_fd(), I2C_SMBUS, &mut args as *mut _) };
        if ret < 0 {
            return Err(std::io::Error::last_os_error());
        }

        Ok(())
    }

    /// Read a 16-bit word from `register` via SMBus word-data protocol.
    ///
    /// The returned value is in host byte order as the kernel performs
    /// the endian swap for standard SMBus word reads.
    pub fn read_word_data(&self, register: u8) -> std::io::Result<u16> {
        let mut data = I2cSmbusData { word: 0 };
        let mut args = I2cSmbusIoctlData {
            read_write: I2C_SMBUS_READ,
            command: register,
            size: I2C_SMBUS_WORD_DATA,
            data: &mut data,
        };

        let ret = unsafe { libc::ioctl(self.file.as_raw_fd(), I2C_SMBUS, &mut args as *mut _) };
        if ret < 0 {
            return Err(std::io::Error::last_os_error());
        }

        Ok(unsafe { data.word })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_nonexistent_bus_returns_error() {
        // Bus 255 should not exist on any real system
        let result = SmbusDevice::open(255, 0x50);
        assert!(result.is_err());
    }

    #[test]
    fn ioctl_data_layout_sizes() {
        // Sanity-check that the repr(C) structs have expected alignment
        assert!(std::mem::size_of::<I2cSmbusData>() >= 34);
        assert!(std::mem::size_of::<I2cSmbusIoctlData>() > 0);
    }
}
