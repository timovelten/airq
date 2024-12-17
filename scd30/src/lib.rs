// Need std for testing
#![cfg_attr(not(test), no_std)]

// Disallow unwrap because it uses core fmt which is large
#![deny(clippy::unwrap_used)]

use crc::{Crc, CRC_8_NRSC_5};
use embedded_hal_async::i2c::{ErrorType, I2c};
use num_enum::IntoPrimitive;

// See https://sensirion.com/media/documents/D7CEEF4A/6165372F/Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
#[repr(u16)]
#[derive(Copy, Clone, PartialEq, Eq, IntoPrimitive)]
#[allow(dead_code)]
enum Command {
    GetDataReadyStatus = 0x0202,
    ReadMeasurement = 0x0300,
    SetTemperatureOffset = 0x5403,
    SetAltitudeCompensation = 0x5102,
    ReadFirmwareVersion = 0xD100,
    Reset = 0xD304,
}

impl Command {
    const fn bytes(self) -> [u8; 2] {
        (self as u16).to_be_bytes()
    }

    #[allow(dead_code)]
    const fn with_data<const M: usize>(self, data: &[u8]) -> [u8; M] {
        let mut ret = [0; M];
        let [x, y] = (self as u16).to_be_bytes();
        ret[0] = x;
        ret[1] = y;

        let mut i = 0;
        while i < data.len() {
            ret[i + 2] = data[i];
            i += 1;
        }

        ret
    }
}

#[derive(Debug, defmt::Format)]
pub enum Error<P> {
    ChecksumMismatch,
    Protocol(P),
}

pub type Result<T, B> = core::result::Result<T, Error<<B as ErrorType>::Error>>;

impl<P> From<P> for Error<P> {
    fn from(value: P) -> Self {
        Self::Protocol(value)
    }
}

pub struct Scd30<B, D> {
    bus: B,
    delay: D,
}

const CRC: Crc<u8> = Crc::<u8>::new(&CRC_8_NRSC_5);

fn crc(data: &[u8]) -> u8 {
    let mut digest = CRC.digest();
    digest.update(data);
    digest.finalize()
}

#[derive(Default, Debug, defmt::Format)]
pub struct Measurement {
    pub co2_concentration: f32,
    pub temperature: f32,
    pub relative_humidity: f32,
}

const ADR: u8 = 0x61;

fn check_crc(data: &[u8]) -> bool {
    for chunk in data.chunks_exact(3) {
        if crc(chunk) != 0 {
            return false;
        }
    }

    true
}

impl<B: I2c, D: embedded_hal_async::delay::DelayNs> Scd30<B, D> {
    pub fn new(bus: B, delay: D) -> Self {
        Self { bus, delay }
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), B> {
        self.bus.write(ADR, data).await?;
        Ok(())
    }

    async fn read<const N: usize>(&mut self) -> Result<[u8; N], B> {
        let mut ret = [0; N];
        self.bus.read(ADR, &mut ret).await?;
        Ok(ret)
    }

    pub async fn read_firmware_version(&mut self) -> Result<(u8, u8), B> {
        self.write(&Command::ReadFirmwareVersion.bytes()).await?;
        let ret: [u8; 3] = self.read().await?;
        if !check_crc(&ret) {
            return Err(Error::ChecksumMismatch);
        }

        Ok((ret[0], ret[1]))
    }

    pub async fn read_measurement(&mut self) -> Result<Measurement, B> {
        self.write(&Command::ReadMeasurement.bytes()).await?;

        // TODO: Use data ready as the datasheet wants us to?
        self.delay.delay_ms(10).await;

        let data: [u8; (4 + 2) * 3] = self.read().await?;
        if !check_crc(&data) {
            return Err(Error::ChecksumMismatch);
        }

        // This is kind of ugly, but what can you do (still better than having a bunch of random
        // looking indices). Of course, if it was more than three values one might want to use a loop
        // or something...
        let [c1, c2, _, c3, c4, _, t1, t2, _, t3, t4, _, rh1, rh2, _, rh3, rh4, _] = data;

        let ret = Measurement {
            co2_concentration: f32::from_be_bytes([c1, c2, c3, c4]),
            temperature: f32::from_be_bytes([t1, t2, t3, t4]),
            relative_humidity: f32::from_be_bytes([rh1, rh2, rh3, rh4]),
        };

        Ok(ret)
    }
}

#[cfg(test)]
mod test {
    use crate::check_crc;

    #[test]
    fn crc_works() {
        let data = [
            0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F, 0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5, 0x42, 0x43,
            0xBF, 0x3A, 0x1B, 0x74,
        ];

        assert!(check_crc(&data));
    }
}
