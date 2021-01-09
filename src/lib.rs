use embedded_hal as hal;

use hal::blocking::delay::{DelayMs};
use hal::blocking::i2c::{Read, Write, WriteRead};

use sensirion_i2c::{crc8, i2c};

const SGPC3_PRODUCT_TYPE: u8 = 1;
const SGPC3_CMD_MEASURE_TEST_OK: u16 = 0xd400;


/// Sgpc3 errors
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2c(E),
    /// CRC checksum validation failed
    Crc,
    ///Self-test measure failure
    SelfTest,
}


impl<E, I2cWrite, I2cRead> From<i2c::Error<I2cWrite, I2cRead>> for Error<E>
where
    I2cWrite: Write<Error = E>,
    I2cRead: Read<Error = E>,
{
    fn from(err: i2c::Error<I2cWrite, I2cRead>) -> Self {
        match err {
            i2c::Error::Crc => Error::Crc,
            i2c::Error::I2cWrite(e) => Error::I2c(e),
            i2c::Error::I2cRead(e) => Error::I2c(e),
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum Command {
    /// Return the serial number.
    GetSerial,
    /// Acquires the supported feature set.
    GetFeatureSet,
    /// Run an on-chip self-test.
    SelfTest,
    /// Initialize 0 air quality measurements.
    InitAirQuality0,
    /// Initialize 64 air quality measurements.
    InitAirQuality64,
    /// Initialize continuous air quality measurements.
    InitAirQualityContinuous,
    /// Get a current air quality measurement.
    MeasureAirQuality,
    /// Measure raw signal.
    MeasureRaw,
    /// Return the baseline value.
    GetAirQualityBaseline,
    /// Return inceptive baseline value.
    GetAirQualityInceptiveBaseline,
    /// Measure air quality raw
    MeasureAirQualityRaw,
    /// Set the baseline value.
    SetBaseline,
    /// Set the current absolute humidity.
    SetHumidity,
    /// Setting power mode.
    SetPowerMode
}

impl Command {
    /// Command and the requested delay in ms
    fn as_tuple(self) -> (u16, u32) {
        match self {
            Command::GetSerial => (0x3682, 1), // This could have been 0.5ms
            Command::GetFeatureSet => (0x202f, 1),
            Command::SelfTest => (0x2032, 220),
            Command::InitAirQuality0 => (0x2089, 10),
            Command::InitAirQuality64 => (0x2003, 10),
            Command::InitAirQualityContinuous => (0x20ae, 10),
            Command::MeasureAirQuality => (0x2008, 50),
            Command::MeasureRaw => (0x204d, 50),
            Command::GetAirQualityBaseline => (0x2015, 10),
            Command::GetAirQualityInceptiveBaseline => (0x20b3, 10),
            Command::MeasureAirQualityRaw => (0x2046, 50),
            Command::SetBaseline => (0x201e, 10),
            Command::SetHumidity => (0x2061, 10),
            Command::SetPowerMode => (0x209f, 10)
        }
    }
}

#[derive(Debug)]
pub struct FeatureSet {
    product_type: u8,
    product_version: u8
}

#[derive(Debug, Default)]
pub struct Sgpc3<I2C,D> {
    i2c: I2C,
    address: u8,
    delay: D
}

impl<I2C, D, E> Sgpc3<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        Sgpc3 {
            i2c,
            address,
            delay,
        }
    }

    /// Acquires the sensor serial number.
    ///
    /// Sensor serial number is only 48-bits long so the remaining 16-bits are zeros.
    pub fn serial(&mut self) -> Result<u64, Error<E>> {
        let mut serial = [0; 9];

        self.delayed_read_cmd(Command::GetSerial, &mut serial)?;

        let serial = u64::from(serial[0])<<40| u64::from(serial[1])<<32|
                     u64::from(serial[3])<<24| u64::from(serial[4])<<16|
                     u64::from(serial[6])<<8| u64::from(serial[7]);
        Ok(serial)
    }

    /// Gets the sensor product type and supported feature set.
    ///
    /// The sensor uses feature versioning system to indicate the device capabilities.
    /// Feature set 5 enables getting TVOC inceptive baseline.
    /// Feature set 6 and above enables ultra-low power-save, setting absolute humidity and preheating.
    /// The behaviour is undefined when using these functions with sensor not supporting the specific features.
    pub fn get_feature_set(&mut self) -> Result<FeatureSet, Error<E>> {
        let mut data = [0; 6];

        self.delayed_read_cmd(Command::GetFeatureSet, &mut data)?;

        let product_type = data[0] >> 4;

        // This is great way to check if the integration and connection is working to sensor.
        assert!(product_type == SGPC3_PRODUCT_TYPE);

        Ok(FeatureSet {
                product_type,
                product_version: data[1]
            }
        )
    }

    /// Sets sensor into ultra-low power mode.
    ///
    /// The SGPC3 offers two operation modes with different power consumptions and sampling intervals. The low-power mode with
    /// 1mA average current and 2s sampling interval and the ultra-low power mode with 0.065mA average current and 30s sampling
    /// interval. By default, the SGPC3 is using the low-power mode. You want to stick with the sensor sampling internal so
    /// you want to take the samples per the internal. The current SW implementation sees ultra low-power mode as
    /// one-way street and once entered, one can get only get out of it through resetting the sensor.
    #[inline]
    pub fn set_ultra_power_mode(&mut self) -> Result<(), Error<E>> {
        let power_mode: [u8;2] = [0;2];

        self.write_command_with_args(Command::SetPowerMode, &power_mode)
    }

    /// Sensor self-test.
    ///
    /// Performs sensor self-test. This is intended for production line and testing and verification only and
    /// shouldn't be needed for normal use. It should not be used after having issues any init commands.
    pub fn self_test(&mut self) -> Result<&mut Self, Error<E>> {
        let mut data = [0;3];
        self.delayed_read_cmd(Command::SelfTest, &mut data)?;

        let result = u16::from_be_bytes([data[0],data[1]]);

        if result != SGPC3_CMD_MEASURE_TEST_OK {
            Err(Error::SelfTest)
        }
        else {
            Ok(self)
        }
    }

    fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd)?;
        i2c::read_words_with_crc(&mut self.i2c, self.address, data)?;
        Ok(())
    }

    fn write_command_with_args(&mut self, cmd: Command, data: &[u8]) -> Result<(), Error<E>> {
        const MAX_TX_BUFFER: usize = 8;

        let mut transfer_buffer = [0; MAX_TX_BUFFER];

        let size = data.len();

        // 2 for command, size of transferred bytes and CRC per each two bytes.
        assert!(size < 2+size+size/2);
        let (command, delay) = cmd.as_tuple();

        transfer_buffer[0..2].copy_from_slice(&command.to_be_bytes());
        let slice = &data[..2];
        transfer_buffer[2..4].copy_from_slice(slice);
        transfer_buffer[4] = crc8::calculate(slice);

        let transfer_buffer = if size > 2 {
            let slice = &data[2..4];
            transfer_buffer[5..7].copy_from_slice(slice);
            transfer_buffer[7] = crc8::calculate(slice);
            &transfer_buffer[..]
        }
        else {
            &transfer_buffer[0..5]
        };

        self.i2c.write(self.address, transfer_buffer).map_err(Error::I2c)?;
        self.delay.delay_ms(delay);

        Ok(())
    }

    fn write_command(&mut self, cmd: Command) -> Result<(), Error<E>> {
        let (command, delay) = cmd.as_tuple();
        i2c::write_command(&mut self.i2c, self.address, command).map_err(Error::I2c)?;
        self.delay.delay_ms(delay);
        Ok(())
    }

    /// Initializes the sensor without preheat.
    ///
    /// Initializing without preheat will lead the early samples to be inaccurate. It is the
    /// responsibility of the caller to wait the sufficient preheat period.
    #[inline]
    pub fn init_no_preheat(&mut self) -> Result<&mut Self, Error<E>> {
        self.write_command(Command::InitAirQuality0)?;
        Ok(self)
    }

    /// Initializes the sensor with preheat.
    ///
    /// This is the standard way of initializing the system.
    #[inline]
    pub fn init_preheat(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::InitAirQualityContinuous)
    }

    /// Initializes the sensor with preheat for feature set 5 sensors
    ///
    /// This is the standard way of initializing the systems with feature set 5 sensor firmware
    #[inline]
    pub fn init_preheat_64s_fs5(&mut self) -> Result<(), Error<E>> {
        self.write_command(Command::InitAirQuality64)
    }

    /// Sets the absolute humidity for the best accuracy.
    ///
    /// The argument must be supplied at fixed-point 8.8bit format.
    #[inline]
    pub fn set_absolute_humidity(&mut self, abs_hum: u32) -> Result<&mut Self, Error<E>> {
        assert!(abs_hum <= 256000);

        // This is Sensirion approximation for performing fixed-point 8.8bit number conversion
        let scaled = ((abs_hum * 16777) >> 16) as u16;

        self.write_command_with_args(Command::SetHumidity, &scaled.to_be_bytes())?;
        Ok(self)
    }

    /// Sets the relative humidity for the best accuracy.
    ///
    /// The arguments are supplied as milli-units. Eg. 20% relative humidity is supplied as 20000
    /// and temperature t_mc as Celsius. 10C is 10000.
    pub fn set_relative_humidity(&mut self, rh: i32, t_mc: i32) -> Result<&mut Self, Error<E>> {
        //let
        let t = (t_mc as f32) / 1000_f32;
        let rh = (rh as f32) / 1000_f32;

        // 61.12 is typically 6.12 but as we need to use % (div by 100) and later scale by multiplying by 1000,
        // we just moved the dot one to left.
        let abs_hum  = 216.7_f32 * (rh*61.12_f32*((17.62_f32*t / (243.12_f32+t)).exp()) / (273.15_f32+t));
        self.set_absolute_humidity(abs_hum as u32)
    }

    /// Measures both TVOC and RAW signal.
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    /// The values are returned as tuple (TVOC, RAW)
    pub fn measure_tvoc_and_raw(&mut self) -> Result<(u16, u16), Error<E>> {
        let mut buffer = [0;6];
        self.delayed_read_cmd(Command::MeasureAirQualityRaw,&mut buffer)?;

        let raw_signal = u16::from_be_bytes([buffer[0],buffer[1]]);
        let tvoc_ppb = u16::from_be_bytes([buffer[3],buffer[4]]);
        Ok((tvoc_ppb, raw_signal))
    }

    /// Measures TVOC
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    pub fn measure_tvoc(&mut self) -> Result<u16,Error<E>> {
        let mut buffer = [0;3];
        self.delayed_read_cmd(Command::MeasureAirQuality,&mut buffer)?;
        let tvoc_ppb = u16::from_be_bytes([buffer[0],buffer[1]]);
        Ok(tvoc_ppb)
    }

    /// Measures RAW signal
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    /// Typically, the caller shouldn't need RAW value but should use TVOC instead.
    pub fn measure_raw(&mut self) -> Result<u16,Error<E>> {
        let mut buffer = [0;3];
        self.delayed_read_cmd(Command::MeasureRaw,&mut buffer)?;
        let raw = u16::from_be_bytes([buffer[0],buffer[1]]);
        Ok(raw)
    }

    /// Acquired the baseline for faster accurate sampling.
    ///
    /// Baseline can be used to reach faster accurate repeatable samples.
    /// Sensor must be supporting feature set 6 for the support.
    /// Check sensor application note for the usage as you need ensure that
    /// sensor has been operating long-enough for valid baseline.
    pub fn get_tvoc_baseline(&mut self) -> Result<u16,Error<E>> {
        let mut buffer = [0;3];
        self.delayed_read_cmd(Command::GetAirQualityBaseline,&mut buffer)?;
        let baseline = u16::from_be_bytes([buffer[0],buffer[1]]);
        Ok(baseline)
    }

    /// Acquired the inceptive baseline for faster accurate sampling.
    ///
    /// Baseline can be used to reach faster accurate repeatable samples.
    /// This method needs to be used for sensors only supporting feature set 5 instead
    /// of using get_tvoc_baseline.
    ///
    /// Check sensor application note for the usage as you need ensure that
    /// sensor has been operating long-enough for valid baseline.
    pub fn get_tvoc_inceptive_baseline(&mut self) -> Result<u16,Error<E>> {
        let mut buffer = [0;3];
        self.delayed_read_cmd(Command::GetAirQualityInceptiveBaseline,&mut buffer)?;
        let baseline = u16::from_be_bytes([buffer[0],buffer[1]]);
        Ok(baseline)
    }


    /// Sets the baseline for faster accurate.
    ///
    /// Baseline will ensure that you can start regarding the accuracy where you left it
    /// off after powering down or reseting the sensor.
    #[inline]
    pub fn set_tvoc_baseline(&mut self, baseline: u16) -> Result<&mut Self,Error<E>> {
        self.write_command_with_args(Command::SetBaseline,&baseline.to_be_bytes())?;
        Ok(self)
    }

}

// Testing is focused on checking the primitive transactions. It is assumed that during
// the real sensor testing, the basic flows in the command structure has been caught.
#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::delay::MockNoop as DelayMock;
    use self::hal::i2c::{Mock as I2cMock, Transaction};
    use super::*;

    /// Test the `serial` function
    #[test]
    fn serial() {
        let (cmd, _) = Command::GetSerial.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0xde, 0xad, 0x98, 0xbe, 0xef, 0x92, 0xde, 0xad, 0x98]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);
        let serial = sensor.serial().unwrap();
        assert_eq!(serial, 0x00deadbeefdead);
    }

    #[test]
    fn selftest_ok() {
        let (cmd, _) = Command::SelfTest.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0xD4, 0x00, 0xC6]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);
        assert!(sensor.self_test().is_ok());
    }

    #[test]
    fn measure_tvoc_and_raw() {
        let (cmd, _) = Command::MeasureAirQuality.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0x12, 0x34, 0x37, 0xbe, 0xef, 0x92]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);
        let (tvoc, raw) = sensor.measure_tvoc_and_raw().unwrap();
        assert_eq!(tvoc, 0x1234);
        assert_eq!(raw, 0xbeef);
    }
}