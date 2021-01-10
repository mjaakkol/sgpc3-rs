use hal::{Delay, I2cdev};
/// Example that runs on Sensirion SGPC3 evaluation board that has temperature and humidity
/// sensors in addition to SGPC.
///
/// If you have the full evaluation board, remove the comments to start using the temperature
/// and humidity sensors.
use linux_embedded_hal as hal;

//use shtcx::PowerMode;

use std::thread;
use std::time::Duration;

use sgpc3::Sgpc3;

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    //let dev_tmp = I2cdev::new("/dev/i2c-1").unwrap();

    let mut sensor = Sgpc3::new(dev, 0x58, Delay);
    //let mut sht = shtcx::shtc3(dev_tmp);
    //let device_id = sht.device_identifier().unwrap();
    //println!("SHT sensor id is {:x}", device_id);

    let serial = sensor.serial().unwrap();

    println!("Serial number is {:x}", serial);

    let feature_set = sensor.get_feature_set().unwrap();
    println!("Feature set {:?}", feature_set);

    sensor.self_test().unwrap();

    //let combined = sht.measure(PowerMode::NormalMode, &mut Delay).unwrap();

    //let (t, rh) = (combined.temperature.as_millidegrees_celsius(), combined.humidity.as_millipercent());

    //println!("Combined: {} °C / {} %RH", t, rh);

    //sensor.set_relative_humidity(rh, t).unwrap();

    sensor.init_preheat().unwrap();

    thread::sleep(Duration::new(184_u64, 0));

    for _ in 0..9 {
        sensor.measure_tvoc().unwrap();
        thread::sleep(Duration::new(2_u64, 0));
    }

    loop {
        //let combined = sht.measure(PowerMode::NormalMode, &mut Delay).unwrap();
        //let (t, rh) = (combined.temperature.as_millidegrees_celsius(), combined.humidity.as_millipercent());

        //println!("Combined: {} °C / {} %RH", t, rh);
        //sensor.set_relative_humidity(rh, t).unwrap();

        let (tvoc, raw) = sensor.measure_tvoc_and_raw().unwrap();

        println!("TVOC: {} / RAW: {}", tvoc, raw);

        thread::sleep(Duration::new(2_u64, 0));

        let raw = sensor.measure_raw().unwrap();
        println!("RAW {}", raw);

        thread::sleep(Duration::new(2_u64, 0));

        let tvoc = sensor.measure_tvoc().unwrap();
        println!("TVOC {}", tvoc);

        let baseline = sensor.get_baseline().unwrap();
        println!("Baseline {}", baseline);

        thread::sleep(Duration::new(2_u64, 0));
    }
}
