use embedded_io_async::{BufRead, ErrorType, Read as _, Write};
use num_enum::{FromPrimitive, IntoPrimitive};

// This is perhaps a bit overengineered considering the fact that a single frame fits into a 520 byte buffer.
// But implementing state machines by hand is always fun, right? :)

/// MOSI = Master Out Slave In, i.e. these are the frames we send
#[derive(Default)]
#[non_exhaustive]
pub struct MosiFrame<const N: usize> {
    pub adr: u8,
    pub cmd: u8,
    pub tx: heapless::Vec<u8, N>,
}

impl<const N: usize> MosiFrame<N> {
    pub async fn send<W: embedded_io_async::Write>(
        &self,
        dest: W,
    ) -> Result<(), ShdlcError<W::Error>> {
        const {
            assert!(N <= 255);
        }

        let mut writer = FrameWriter::new(dest);
        writer
            .write_all(&[self.adr, self.cmd, self.tx.len() as u8])
            .await?;
        writer.write_all(&self.tx).await?;
        writer.write_checksum().await?;
        writer.finish().await?;

        Ok(())
    }
}

/// MISO = Master In Slave Out, i.e. these are the frames we receive
#[derive(Debug)]
pub struct MisoFrame {
    #[allow(dead_code)]
    pub adr: u8,
    #[allow(dead_code)]
    pub cmd: u8,
    pub state: u8,
    pub rx: heapless::Vec<u8, 255>,
}

impl MisoFrame {
    /// If the length exceeds the capacity `N` the data will be truncated.
    pub async fn recv<R: BufRead>(src: &mut R) -> Result<MisoFrame, ShdlcError<R::Error>> {
        let mut reader = FrameReader::new(src);
        let mut ret = MisoFrame {
            adr: reader.expect_byte().await?,
            cmd: reader.expect_byte().await?,
            state: reader.expect_byte().await?,
            rx: Default::default(),
        };

        let mut cursor = 0;

        let l = reader.expect_byte().await? as usize;
        ret.rx.resize(l, 0).unwrap();

        while cursor < ret.rx.len() {
            let read = reader.read(&mut ret.rx[cursor..]).await?;
            if read == 0 {
                return Err(ShdlcError::ExpectedByte);
            }

            cursor += read;
        }

        let computed_checksum = reader.computed_checksum();
        let checksum = reader.expect_byte().await?;
        if computed_checksum != checksum {
            return Err(ShdlcError::ChecksumMismatch);
        }

        reader.finish().await?;

        Ok(ret)
    }

    pub fn state(&self) -> MisoState {
        self.state.into()
    }
}

/// Table 6 in the datahsheet
#[repr(u8)]
#[derive(FromPrimitive, IntoPrimitive, Clone, Copy, PartialEq, Eq, Hash, Debug, defmt::Format)]
pub enum MisoState {
    Ok = 0x00,
    WrongLength = 0x01,
    UnknownCommand = 0x02,
    Denied = 0x03,
    IllegalParamter = 0x04,
    InternalOutOfRange = 0x28,
    NotAllowedInCurrentState = 0x43,
    #[num_enum(catch_all)]
    #[allow(dead_code)]
    Unknown(u8),
}

impl MisoState {
    pub fn is_ok(self) -> bool {
        self == Self::Ok
    }
}

enum WriterState {
    Nothing,
    SentStart,
    Done,
}

/// Write a single SHDLC frame
pub struct FrameWriter<W> {
    inner: W,
    state: WriterState,
    checksum: u8,
}

impl<W: embedded_io_async::Write> FrameWriter<W> {
    pub fn new(inner: W) -> Self {
        Self {
            inner,
            state: WriterState::Nothing,
            checksum: 0,
        }
    }

    pub async fn write_checksum(&mut self) -> Result<u8, <Self as ErrorType>::Error> {
        // As far as I understand the docs, it is possible for the checksum to be equal
        // to one of the escape characters (and in that case we have to escape it)
        let checksum = !self.checksum;
        self.write(&[checksum]).await?;

        Ok(checksum)
    }

    pub async fn finish(&mut self) -> Result<(), <Self as ErrorType>::Error> {
        self.inner.write(&[0x7e]).await?;
        self.state = WriterState::Done;
        Ok(())
    }

    fn bytes_for_checksum(&mut self, buf: &[u8]) {
        for &b in buf {
            self.checksum = self.checksum.wrapping_add(b);
        }
    }
}

impl<W: embedded_io_async::Write> ErrorType for FrameWriter<W> {
    type Error = ShdlcError<W::Error>;
}

impl<W: embedded_io_async::Write> embedded_io_async::Write for FrameWriter<W> {
    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush().await?;
        Ok(())
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Write starting data
        match self.state {
            WriterState::Done => return Err(ShdlcError::Done),
            WriterState::SentStart => {}
            WriterState::Nothing => {
                // Send start
                self.inner.write(&[0x7e]).await?;
                self.state = WriterState::SentStart;
            }
        }

        let ctrl_index = buf
            .iter()
            .position(|x| matches!(x, 0x7e | 0x7d | 0x11 | 0x13));
        if let Some(i) = ctrl_index {
            // Try and write data that is free of ctrl index
            let rem = &buf[..i];
            self.inner.write_all(rem).await?;
            self.bytes_for_checksum(rem);

            // Write the control byte itself
            let cs = match buf[i] {
                0x7e => [0x7d, 0x5e],
                0x7d => [0x7d, 0x5d],
                0x11 => [0x7d, 0x31],
                0x13 => [0x7d, 0x33],
                _ => unreachable!(),
            };
            self.inner.write_all(&cs).await?;
            self.bytes_for_checksum(&buf[i..i + 1]);

            Ok(i + 1)
        } else {
            self.inner.write_all(buf).await?;
            self.bytes_for_checksum(buf);
            Ok(buf.len())
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum ReaderState {
    ExpectStart,
    ExpectEscapedByte,
    Reading,
    Done,
}

struct FrameReader<R> {
    inner: R,
    state: ReaderState,
    checksum: u8,
}

impl<R: embedded_io_async::BufRead> FrameReader<R> {
    fn new(inner: R) -> Self {
        Self {
            inner,
            state: ReaderState::ExpectStart,
            checksum: 0,
        }
    }

    pub async fn read_byte(&mut self) -> Result<Option<u8>, <Self as ErrorType>::Error> {
        let ret = loop {
            let buf = self.inner.fill_buf().await?;

            // We go byte by byte anway. But note that we do not necessarily consume this byte
            let b = buf.first().copied();

            match (self.state, b) {
                (ReaderState::Done, _) => break None,
                (ReaderState::ExpectStart, Some(0x7e)) => {
                    // Expect start and get start!
                    self.inner.consume(1);
                    self.state = ReaderState::Reading;

                    // Do not break. Continue and actually get some data
                },
                (ReaderState::ExpectStart, _) => return Err(ShdlcError::ExpectedStart),
                (ReaderState::Reading, None) => return Err(ShdlcError::ExpectedEnd),
                (ReaderState::Reading, Some(b)) => {
                    self.inner.consume(1);

                    match b {
                        0x7e => {
                            self.state = ReaderState::Done;
                            break None
                        },
                        0x7d => {
                            self.state = ReaderState::ExpectEscapedByte;
                            continue
                        },
                        b => break Some(b)
                    }
                }
                (ReaderState::ExpectEscapedByte, b) => {
                    let unescaped = match b {
                        Some(0x5e) => 0x7e,
                        Some(0x5d) => 0x7d,
                        Some(0x31) => 0x11,
                        Some(0x33) => 0x13,
                        _ => return Err(ShdlcError::ExpectedEscapedByte)
                    };
                    self.inner.consume(1);

                    self.state = ReaderState::Reading;
                    break Some(unescaped);
                }
            }
        };

        if let Some(ret) = ret {
            self.checksum = self.checksum.wrapping_add(ret);
        }

        Ok(ret)
    }

    pub fn computed_checksum(&self) -> u8 {
        !self.checksum
    }

    async fn expect_byte(&mut self) -> Result<u8, <Self as ErrorType>::Error> {
        match self.read_byte().await? {
            Some(b) => Ok(b),
            None => Err(ShdlcError::ExpectedByte),
        }
    }

    /// Consume all remaining bytes of the current frame
    pub async fn finish(&mut self) -> Result<(), <Self as ErrorType>::Error> {
        loop {
            if self.state == ReaderState::Done {
                return Ok(())
            }

            let _ = self.read_byte().await?;
        }
    }
}

impl<R: embedded_io_async::BufRead> embedded_io::ErrorType for FrameReader<R> {
    type Error = ShdlcError<R::Error>;
}

impl<R: embedded_io_async::BufRead> embedded_io_async::Read for FrameReader<R> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        #[allow(clippy::needless_range_loop)]
        for i in 0..buf.len() {
            match self.read_byte().await? {
                Some(b) => buf[i] = b,
                None => return Ok(i),
            }
        }

        Ok(buf.len())
    }
}

/// It is convenient for us to only use one error type for both the sender and the receiver
#[derive(Debug, defmt::Format)]
pub enum ShdlcError<E> {
    /// Expected to read the start (0x7e) of a frame but got something else
    ExpectedStart,
    /// Expected to read the end (0x7e) of a frame but got something else
    ExpectedEnd,
    /// Expected to read one of the four escaped characters
    ExpectedEscapedByte,
    /// Expected to read more data but got nothing
    ExpectedByte,
    ChecksumMismatch,
    /// Writer is done and will not accept any more data
    Done,
    Other(E),
}

impl<E> From<E> for ShdlcError<E> {
    fn from(value: E) -> Self {
        Self::Other(value)
    }
}

impl<E: embedded_io::Error> embedded_io::Error for ShdlcError<E> {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Other(err) => err.kind(),
            Self::Done => embedded_io::ErrorKind::WriteZero,
            _ => embedded_io::ErrorKind::Other,
        }
    }
}

#[cfg(test)]
mod test {
    use super::{MosiFrame, ShdlcError};

    use super::{FrameReader, FrameWriter, MisoFrame};
    use assert_matches::assert_matches;
    use embedded_io::{Read, Write};
    use embedded_io_async::{Read as _, Write as _};
    use macro_rules_attribute::apply;
    use rand::{rngs::SmallRng, Rng as _, RngCore, SeedableRng};
    use smol_macros::test;

    #[test_log::test(apply(test!))]
    async fn unexpected_end() {
        let msg = vec![0x7e];
        let err = MisoFrame::recv(&mut &msg[..]).await.unwrap_err();
        assert_matches!(err, ShdlcError::ExpectedEnd);
    }

    #[test_log::test(apply(test!))]
    async fn expected_escape_char() {
        let msg = vec![0x7e, 0x7d];
        let err = MisoFrame::recv(&mut &msg[..]).await.unwrap_err();
        assert_matches!(err, ShdlcError::ExpectedEscapedByte);
    }

    #[test_log::test(apply(test!))]
    async fn expected_only_escape_char() {
        let msg = vec![0x7e, 0x7d, 0xaa];
        let err = MisoFrame::recv(&mut &msg[..]).await.unwrap_err();
        assert_matches!(err, ShdlcError::ExpectedEscapedByte);
    }

    /// Send some data using FrameWriter, then read it back using FrameReader. Make sure that
    /// it arrives correctly, and that the reader receives a correct checksum at the end.
    async fn send_and_read(src: &[u8], checksum: Option<u8>) {
        let mut phys = Vec::new();

        // Write data, with checksum as suffix
        let mut writer = FrameWriter::new(&mut phys);
        writer.write_all(src).await.unwrap();
        let check = writer.write_checksum().await.unwrap();
        writer.finish().await.unwrap();

        println!("Written checksum is {check}");
        println!("Written data is {:x?}", phys);

        // Read data and verify checksum
        let mut hopefully_src = vec![0; src.len()];
        let mut reader = FrameReader::new(&phys[..]);

        let mut cursor = 0;
        while cursor < hopefully_src.len() {
            let read = reader.read(&mut &mut hopefully_src[cursor..]).await.unwrap();
            if read == 0 {
                panic!("Not enough data");
            }
            cursor += read;
        }

        println!("{:x?}", hopefully_src);
        assert_eq!(src, hopefully_src, "{src:0x?} != {hopefully_src:x?}");

        // Read checksum
        let computed_checksum = reader.computed_checksum();
        let read_checksum = reader.expect_byte().await.unwrap();
        assert!(computed_checksum == read_checksum);

        if let Some(crc) = checksum {
            assert_eq!(crc, read_checksum);
        }
    }

    #[test_log::test(apply(test!))]
    async fn send_read_1() {
        send_and_read(&[0x43, 0x11, 0x7F], None).await;
    }

    #[test_log::test(apply(test!))]
    async fn send_read_2() {
        send_and_read(&[0x00, 0x00, 0x02, 0x01, 0x03], Some(0xf9)).await;
    }

    #[test_log::test(apply(test!))]
    async fn send_read_3() {
        send_and_read(
            &[0x7e, 0x13, 0xe7, 0xe7, 0x6e, 0x32, 0x56, 0x74, 0xca, 0x21],
            None,
        ).await;
    }

    #[test_log::test(apply(test!))]
    async fn back_to_back() {
        let mut msg = Vec::new();
        let frames: [&'static [u8]; 3] = [
            &[0x00, 0x00, 0x00, 0x00, 0xFF],
            &[0x00, 0x00, 0x00, 0x00, 0xFF],
            &[0x00, 0xD0, 0x00, 0x09, 0x30, 0x30, 0x30, 0x38, 0x30, 0x30, 0x30, 0x30, 0x00]
        ];

        for frame in frames {
            let mut writer = FrameWriter::new(&mut msg);
            writer.write_all(frame).await.unwrap();
            writer.write_checksum().await.unwrap();
            writer.finish().await.unwrap();
        }
        
        let mut reader = &msg[..];
        for _ in 0..3 {
            let _ = MisoFrame::recv(&mut reader).await.unwrap();
        }
    }

    #[apply(test!)]
    async fn send_read_rand() {
        let mut rng = SmallRng::seed_from_u64(112233445566);

        for _ in 0..10_000 {
            let len: usize = rng.gen_range(0..32);
            let mut data = vec![0; len];
            rng.fill_bytes(&mut data);
            println!("Checking with data {:x?}", data);

            send_and_read(&data, None).await;
        }
    }

    #[apply(test)]
    async fn truncates_data() {

    }
}
