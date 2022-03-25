//! Platform agnostic Rust driver for Sensirion SGPC3 gas sensor using
//! the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## Sensirion SGPC3
//!
//! Sensirion SGPC3 is a low-power accurate gas sensor for air quality application.
//! The sensor has different sampling rates to optimize power-consumption per application
//! bases as well as ability save and set the baseline for faster start-up accuracy.
//! The sensor uses I²C interface and measures TVOC (*Total Volatile Organic Compounds*)
//!
//! Datasheet: https://www.sensirion.com/file/datasheet_sgpc3
//!
//! ## Usage
//!
//! ### Instantiating
//!
//! Import this crate and an `embedded_hal` implementation, then instantiate
//! the device:
//!
//! ```no_run
//! use linux_embedded_hal as hal;
//!
//! use hal::{Delay, I2cdev};
//! use sgpc3::Sgpc3;
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sgp = Sgpc3::new(dev, 0x58, Delay);
//!
//! ```
//!
//! ### Fetching Sensor Feature Set
//!
//! Sensor feature set is important to determine the device capabilities.
//! Most new sensors are at level 6 or above. Consult the datasheet for the implications.
//!
//! ```no_run
//! use hal::{Delay, I2cdev};
//! use sgpc3::Sgpc3;
//!
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Sgpc3::new(dev, 0x58, Delay);
//! let feature_set = sensor.get_feature_set().unwrap();
//! println!("Feature set {:?}", feature_set);
//! ```
//!
//! ### Doing Measurements
//!
//! Before you do any measurements, you need to initialize the sensor.
//!
//! ```no_run
//!
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Sgpc3::new(dev, 0x58, Delay);
//! sensor.init_preheat().unwrap();
//!
//! thread::sleep(Duration::new(16_u64, 0));
//!
//! loop {
//!     let tvoc = sensor.measure_tvoc().unwrap();
//!     println!("TVOC {}", tvoc);
//! }

//! ```
//!
//! SGPC3 has few things that you need to keep in mind. The first is pre-heating.
//! The amount of preheating depends on when the sensor was used the last time and
//! if you have the baseline saved. Baseline is adjusted inside the sensor with each measurement
//! and you want to save it eg. each hour in the case you need to reset the sensor and start
//! from the beginning.
//!
//! The recommended initialization flow is:
//! ```no_run,ignore
//! let dev = I2cdev::new("/dev/i2c-1");
//! let mut sensor = Sgpc3::new(dev, 0x58, Delay);
//! sensor.init_preheat();
//! sensor.set_baseline(baseline);
//!
//! thread::sleep(Duration::new(sleep_time, 0));
//! sensor.measure_tvoc();
//! ```
//!
//! The table provides pre-heating times per sensor down-time
//!
//! | Sensor down-time                     | Accelerated warm-up time |
//! |--------------------------------------|--------------------------|
//! |            1 min – 30 min            |             -            |
//! |             30 min – 6 h             |           16 s           |
//! |             6 h – 1 week             |           184 s          |
//! | more than 1 week / initial switch-on |           184 s          |
//!
//! Once the sensor has been taking the sufficient time for pre-heating. You need to call
//! measurement function eg. 'measure_tvoc'. This will enable the sensor to go to sleep and continue initialization.
//! The sensor readings won't change for the first 20s so you might as well discard all of them.
//!
//! You have two operation modes to choose from. Either the standard mode where you read the value
//! every 2s or use the ultra-low power mode where you read the sensor every 30s.
//!
//! If you want to use ultra-power mode, you want to call that prior calling any init-function.
//! Your baseline is power-mode dependent so you don't want to switch back and forth between
//! the power-modes as it always requires re-initialization. Also, the sensor accuracy varies between
//! the modes making the comparison of the values between modes no apples-to-apples anymore.
//! In another words, choose your power-mode per your application and stick with it.
//!
//! After initialization, you are ready to measure TVOC in loop per the selected measurement interval.
//! As said earlier, you want to stick with the internal - no shorter or longer than the defined value.
//!
//! If no stored baseline is available after initializing the baseline algorithm, the sensor has to run for
//! 12 hours until the baseline can be stored. This will ensure an optimal behavior for subsequent startups.
//! Reading out the baseline prior should be avoided unless a valid baseline is restored first. Once the
//! baseline is properly initialized or restored, the current baseline value should be stored approximately
//! once per hour. While the sensor is off, baseline values are valid for a maximum of seven days.
//!
//! SGPC3 is a great sensor and fun to use! I hope your sensor selection and this driver servers you well.
#![cfg_attr(not(test), no_std)]

#[cfg(no_std)]
use num_traits::real::Real;

use embedded_hal as hal;

use hal::blocking::delay::DelayMs;
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
    SetPowerMode,
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
            Command::SetPowerMode => (0x209f, 10),
        }
    }
}

#[derive(Debug)]
pub struct FeatureSet {
    /// Product type for SGPC3 is always 1.
    pub product_type: u8,
    /// Product feature set defines the capabilities of the sensor. Consult datasheet
    /// for the differences as they do impact on how you want to use APIs.
    pub product_featureset: u8,
}

#[derive(Debug, Default)]
pub struct Sgpc3<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
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

        let serial = u64::from(serial[0]) << 40
            | u64::from(serial[1]) << 32
            | u64::from(serial[3]) << 24
            | u64::from(serial[4]) << 16
            | u64::from(serial[6]) << 8
            | u64::from(serial[7]);
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
            product_featureset: data[1],
        })
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
        let power_mode: [u8; 2] = [0; 2];

        self.write_command_with_args(Command::SetPowerMode, &power_mode)
    }

    /// Sensor self-test.
    ///
    /// Performs sensor self-test. This is intended for production line and testing and verification only and
    /// shouldn't be needed for normal use. It should not be used after having issues any init commands.
    pub fn self_test(&mut self) -> Result<&mut Self, Error<E>> {
        let mut data = [0; 3];
        self.delayed_read_cmd(Command::SelfTest, &mut data)?;

        let result = u16::from_be_bytes([data[0], data[1]]);

        if result != SGPC3_CMD_MEASURE_TEST_OK {
            Err(Error::SelfTest)
        } else {
            Ok(self)
        }
    }

    /// Command for reading values from the sensor
    fn delayed_read_cmd(&mut self, cmd: Command, data: &mut [u8]) -> Result<(), Error<E>> {
        self.write_command(cmd)?;
        i2c::read_words_with_crc(&mut self.i2c, self.address, data)?;
        Ok(())
    }

    /// Writes commands with arguments
    fn write_command_with_args(&mut self, cmd: Command, data: &[u8]) -> Result<(), Error<E>> {
        const MAX_TX_BUFFER: usize = 8;

        let mut transfer_buffer = [0; MAX_TX_BUFFER];

        let size = data.len();

        // 2 for command, size of transferred bytes and CRC per each two bytes.
        assert!(size < 2 + size + size / 2);
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
        } else {
            &transfer_buffer[0..5]
        };

        self.i2c
            .write(self.address, transfer_buffer)
            .map_err(Error::I2c)?;
        self.delay.delay_ms(delay);

        Ok(())
    }

    /// Writes commands without additional arguments.
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
        let abs_hum = 216.7_f32
            * (rh * 61.12_f32 * ((17.62_f32 * t / (243.12_f32 + t)).exp()) / (273.15_f32 + t));
        self.set_absolute_humidity(abs_hum as u32)
    }

    /// Measures both TVOC and RAW signal.
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    /// The values are returned as tuple (TVOC, RAW)
    pub fn measure_tvoc_and_raw(&mut self) -> Result<(u16, u16), Error<E>> {
        let mut buffer = [0; 6];
        self.delayed_read_cmd(Command::MeasureAirQualityRaw, &mut buffer)?;

        let raw_signal = u16::from_be_bytes([buffer[0], buffer[1]]);
        let tvoc_ppb = u16::from_be_bytes([buffer[3], buffer[4]]);
        Ok((tvoc_ppb, raw_signal))
    }

    /// Measures TVOC
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    pub fn measure_tvoc(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0; 3];
        self.delayed_read_cmd(Command::MeasureAirQuality, &mut buffer)?;
        let tvoc_ppb = u16::from_be_bytes([buffer[0], buffer[1]]);
        Ok(tvoc_ppb)
    }

    /// Measures RAW signal
    ///
    /// The measurement should be performed at the configured sampling internal for the best accuracy.
    /// Typically, the caller shouldn't need RAW value but should use TVOC instead.
    pub fn measure_raw(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0; 3];
        self.delayed_read_cmd(Command::MeasureRaw, &mut buffer)?;
        let raw = u16::from_be_bytes([buffer[0], buffer[1]]);
        Ok(raw)
    }

    /// Acquired the baseline for faster accurate sampling.
    ///
    /// Baseline can be used to reach faster accurate repeatable samples.
    /// Sensor must be supporting feature set 6 for the support.
    /// Check sensor application note for the usage as you need ensure that
    /// sensor has been operating long-enough for valid baseline.
    pub fn get_baseline(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0; 3];
        self.delayed_read_cmd(Command::GetAirQualityBaseline, &mut buffer)?;
        let baseline = u16::from_be_bytes([buffer[0], buffer[1]]);
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
    pub fn get_inceptive_baseline(&mut self) -> Result<u16, Error<E>> {
        let mut buffer = [0; 3];
        self.delayed_read_cmd(Command::GetAirQualityInceptiveBaseline, &mut buffer)?;
        let baseline = u16::from_be_bytes([buffer[0], buffer[1]]);
        Ok(baseline)
    }

    /// Sets the baseline for faster accurate.
    ///
    /// Baseline will ensure that you can start regarding the accuracy where you left it
    /// off after powering down or reseting the sensor.
    #[inline]
    pub fn set_baseline(&mut self, baseline: u16) -> Result<&mut Self, Error<E>> {
        self.write_command_with_args(Command::SetBaseline, &baseline.to_be_bytes())?;
        Ok(self)
    }

    /// Initialize sensor for use
    ///
    /// Full initialization sequence for common way to initialize the sensor for production use.
    /// This code uses the existing functionality making this shortcut to get things going for
    /// those who don't want to learn the internal workings of the sensor. This method can only
    /// be used with sensors supporting feature set 6 and above.
    ///
    /// It is assumed that ['baseline'] has been stored in system non-volatile memory with timestamp
    /// during the earlier operation. Datasheet says "If no stored baseline is available after initializing
    /// the baseline algorithm, the sensor has to run for 12 hours until the baseline can be stored.
    /// This will ensure an optimal behavior for subsequent startups. Reading out the baseline prior should
    /// be avoided unless a valid baseline is restored first. Once the baseline is properly initialized or
    /// restored, the current baseline value should be stored approximately once per hour. While the sensor
    /// is off, baseline values are valid for a maximum of seven days." Baseline age is provided in seconds
    /// and set value zero if there is no baseline available.
    ///
    /// Initialization can take up to 204s so depending on the application the user may want to run this in own task.
    ///
    /// Once the method is complete, the user should immediately take a sample and then continue taking them
    /// per the defined power-mode. In ultra power-save, the sampling frequency is 30s and in standard mode 2s.
    pub fn initialize(
        &mut self,
        baseline: u16,
        baseline_age_s: u32,
        ultra_power_save: bool,
    ) -> Result<&mut Self, Error<E>> {
        if ultra_power_save {
            self.set_ultra_power_mode()?;
        }

        self.init_preheat()?;

        let sleep_time = if baseline_age_s == 0 || baseline_age_s > 7 * 24 * 60 * 60 {
            // More than week old or initial switch-on
            184 * 1000
        } else {
            self.set_baseline(baseline)?;

            if baseline_age_s > 0 && baseline_age_s <= 30 * 60 {
                // Less than 30min from the last save. This is fresh puppy
                0
            } else if baseline_age_s > 30 * 60 && baseline_age_s <= 6 * 60 * 60 {
                // Less than six hours since the last baseline save
                16 * 1000
            } else {
                // Maximum pre-head time but baseline still valid if less than week old
                184 * 1000
            }
        };

        self.delay.delay_ms(sleep_time);

        // Releases preheat and start the internal sensor initialization
        self.measure_tvoc()?;

        // From the document: "After the accelerated warm-up phase, the initialization takes 20 seconds,
        // during which the IAQ output will not change."
        self.delay.delay_ms(20 * 1000);
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
            Transaction::read(
                0x58,
                vec![0xde, 0xad, 0x98, 0xbe, 0xef, 0x92, 0xde, 0xad, 0x98],
            ),
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
    fn selftest_failed() {
        let (cmd, _) = Command::SelfTest.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0xde, 0xad, 0x98]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);
        assert!(!sensor.self_test().is_ok());
    }

    #[test]
    fn test_crc_error() {
        let (cmd, _) = Command::SelfTest.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0xD4, 0x00, 0x00]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);

        match sensor.self_test() {
            Err(Error::Crc) => {}
            Err(_) => panic!("Unexpected error in CRC test"),
            Ok(_) => panic!("Unexpected success in CRC test"),
        }
    }

    #[test]
    fn measure_tvoc_and_raw() {
        let (cmd, _) = Command::MeasureAirQualityRaw.as_tuple();
        let expectations = [
            Transaction::write(0x58, cmd.to_be_bytes().to_vec()),
            Transaction::read(0x58, vec![0x12, 0x34, 0x37, 0xbe, 0xef, 0x92]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut sensor = Sgpc3::new(mock, 0x58, DelayMock);
        let (tvoc, raw) = sensor.measure_tvoc_and_raw().unwrap();
        assert_eq!(tvoc, 0xbeef);
        assert_eq!(raw, 0x1234);
    }
}
