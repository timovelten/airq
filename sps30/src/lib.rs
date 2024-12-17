// Need std for testing
#![cfg_attr(not(test), no_std)]

// Disallow unwrap because it uses core fmt which is large
#![deny(clippy::unwrap_used)]

use embedded_hal_async::delay::DelayNs;
use heapless::Vec;
use num_enum::IntoPrimitive;
use shdlc::{MisoFrame, MisoState, MosiFrame, ShdlcError};

mod shdlc;

/// Table 7 in the datasheet
#[repr(u8)]
#[derive(IntoPrimitive, Clone, Copy, PartialEq, Eq, Hash)]
enum Command {
    StartMeasurement = 0x00,
    StopMeasurement = 0x01,
    ReadMeasuredValue = 0x03,
    StartFanCleaning = 0x56,
    ReadWriteAutoCleaningInterval = 0x80,
    DeviceInformation = 0xD0,
    ReadVersion = 0xD1,
    ReadDeviceStatusRegister = 0xD2,
    Reset = 0xD3,
    // Note that we do not support Sleep and wakeup for
}

fn as_array<const N: usize>(xs: &[u8]) -> [u8; N] {
    #[allow(clippy::unwrap_used)]
    xs.try_into().unwrap()
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[repr(u8)]
#[derive(IntoPrimitive)]
pub enum DeviceInformation {
    ProductType = 0x00,
    SerialNumber = 0x03,
}
pub struct Sps30<W, D> {
    serial: W,
    delay: D,
}

#[derive(defmt::Format, Debug)]
pub enum Error<S: embedded_io::ErrorType> {
    ShdlcError(ShdlcError<S::Error>),
    InvalidUtf8,
    MisoNotOkay(MisoState),
    InvalidResponse,
}

#[derive(Debug, defmt::Format, Default)]
pub struct Measurement {
    pub mass_pm1: f32,
    pub mass_pm2: f32,
    pub mass_pm4: f32,
    pub mass_pm10: f32,

    pub num_pm0: f32,
    pub num_pm1: f32,
    pub num_pm2: f32,
    pub num_pm4: f32,
    pub num_pm10: f32,

    pub typical_particle_size: f32,
}

type Result<T, S> = core::result::Result<T, Error<S>>;

impl<S, D> Sps30<S, D>
where
    S: embedded_io_async::BufRead + embedded_io_async::Write,
    D: DelayNs,
{
    pub fn new(serial: S, delay: D) -> Self {
        Self { serial, delay }
    }

    async fn recv_response(&mut self) -> Result<MisoFrame, S> {
        let resp = MisoFrame::recv(&mut self.serial)
            .await
            .map_err(Error::ShdlcError)?;
        if !resp.state().is_ok() {
            return Err(Error::MisoNotOkay(resp.state()));
        }

        // TODO: Maybe check that the sent cmd matches?

        Ok(resp)
    }

    async fn send<const N: usize>(&mut self, cmd: Command, data: [u8; N]) -> Result<(), S> {
        let frame: MosiFrame<N> = MosiFrame {
            adr: 0,
            cmd: cmd.into(),
            #[allow(clippy::unwrap_used)]
            tx: Vec::from_slice(&data).unwrap(),
        };

        frame
            .send(&mut self.serial)
            .await
            .map_err(Error::ShdlcError)?;

        Ok(())
    }

    pub async fn reset(&mut self) -> Result<(), S> {
        self.send(Command::Reset, []).await?;
        self.recv_response().await?;

        // Datasheet Table 8 says command execution time is less than 100 ms
        self.delay.delay_ms(100).await;

        Ok(())
    }

    pub async fn start_measurement(&mut self) -> Result<(), S> {
        self.send(Command::StartMeasurement, [0x01, 0x03]).await?;
        self.recv_response().await?;
        Ok(())
    }

    pub async fn stop_measurement(&mut self) -> Result<(), S> {
        self.send(Command::StopMeasurement, []).await?;
        self.recv_response().await?;
        Ok(())
    }

    pub async fn device_status(&mut self, clear: bool) -> Result<u32, S> {
        self.send(Command::ReadDeviceStatusRegister, [clear as u8])
            .await?;
        let resp = self.recv_response().await?;
        let &[a, b, c, d, ..] = resp.rx.as_slice() else {
            return Err(Error::InvalidResponse);
        };
        let ret = u32::from_be_bytes([a, b, c, d]);
        
        Ok(ret)
    }

    pub async fn read_version(&mut self) -> Result<[u8; 7], S> {
        self.send(Command::ReadVersion, []).await?;
        let resp = self.recv_response().await?;
        if resp.rx.len() < 7 {
            return Err(Error::InvalidResponse);
        }

        Ok(as_array(&resp.rx[0..7]))
    }

    pub async fn read_measured_value(&mut self) -> Result<Measurement, S> {
        self.send(Command::ReadMeasuredValue, []).await?;
        let resp = self.recv_response().await?.rx;
        if resp.len() < 40 {
            return Err(Error::InvalidResponse);
        }

        fn f(xs: &[u8]) -> f32 {
            f32::from_be_bytes(as_array(xs))
        }
        
        let ret = Measurement {
            mass_pm1: f(&resp[0..4]),
            mass_pm2: f(&resp[4..8]),
            mass_pm4: f(&resp[8..12]),
            mass_pm10: f(&resp[12..16]),
            num_pm0: f(&resp[16..20]),
            num_pm1: f(&resp[20..24]),
            num_pm2: f(&resp[24..28]),
            num_pm4: f(&resp[28..32]),
            num_pm10: f(&resp[32..36]),
            typical_particle_size: f(&resp[36..40]),
        };

        Ok(ret)
    }

    pub async fn start_fan_cleaning(&mut self) -> Result<(), S> {
        self.send(Command::StartFanCleaning, []).await?;
        self.recv_response().await?;

        Ok(())
    }

    pub async fn read_auto_cleaning_interval(&mut self) -> Result<u32, S> {
        self.send(Command::ReadWriteAutoCleaningInterval, [0x00])
            .await?;
        let resp = self.recv_response().await?;
        let &[a, b, c, d, ..] = resp.rx.as_slice() else {
            return Err(Error::InvalidResponse);
        };

        let ret = u32::from_be_bytes([a, b, c, d]);
        Ok(ret)
    }

    pub async fn write_auto_cleaning_interval(&mut self, seconds: u32) -> Result<(), S> {
        let [a, b, c, d] = seconds.to_be_bytes();
        let data = [0x01, a, b, c, d];
        self.send(Command::ReadWriteAutoCleaningInterval, data)
            .await?;
        self.recv_response().await?;

        todo!()
    }

    pub async fn device_information(
        &mut self,
        info: DeviceInformation,
    ) -> Result<heapless::String<255>, S> {
        self.send(Command::DeviceInformation, [info.into()]).await?;
        let resp = self.recv_response().await?;

        let ret = heapless::String::from_utf8(resp.rx).map_err(|_| Error::InvalidUtf8)?;
        Ok(ret)
    }
}
