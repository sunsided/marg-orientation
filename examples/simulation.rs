use csv::ReaderBuilder;
use serde::de::DeserializeOwned;
use serde::Deserialize;
use std::error::Error;

/// MPU6050 accelerometer and gyroscope readings.
///
/// The deserialization ignores the temperature readings.
#[derive(Debug, Deserialize)]
struct MPU6050 {
    /// The sample time, in seconds, relative to the beginning of the recording.
    #[serde(rename = "sec")]
    time: f32,
    /// Accelerometer reading on the x-axis.
    acc_x: f32,
    /// Accelerometer reading on the y-axis.
    acc_y: f32,
    /// Accelerometer reading on the z-axis.
    acc_z: f32,
    /// Gyroscope reading on the x-axis.
    gyro_x: f32,
    /// Gyroscope reading on the y-axis.
    gyro_y: f32,
    /// Gyroscope reading on the z-axis.
    gyro_z: f32,
}

/// HMC5833L magnetometer readings.
#[derive(Debug, Deserialize)]
struct HMC5833L {
    /// The sample time, in seconds, relative to the beginning of the recording.
    #[serde(rename = "sec")]
    time: f32,
    /// Magnetometer reading on the x-axis.
    compass_x: f32,
    /// Magnetometer reading on the y-axis.
    compass_y: f32,
    /// Magnetometer reading on the z-axis.
    compass_z: f32,
}

pub trait Time {
    /// Gets the sample time.
    fn time(&self) -> f32;
}

impl Time for MPU6050 {
    fn time(&self) -> f32 {
        self.time
    }
}

impl Time for HMC5833L {
    fn time(&self) -> f32 {
        self.time
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mpu6050 =
        read_csv::<MPU6050>("tests/data/sensor-fusion/data/set-2/full-sphere/mpu6050.csv")?;
    let hmc5833l =
        read_csv::<HMC5833L>("tests/data/sensor-fusion/data/set-2/full-sphere/hmc5833l.csv")?;

    // println!("{:#?}", mpu6050[0]);
    // println!("{:#?}", hmc5833l[0]);

    // Determine sample rates.
    let (mpu6050_sample_rate, _) = determine_sampling(&mpu6050);
    let (hmc8533l_sample_rate, _) = determine_sampling(&hmc5833l);

    println!("Average sample rates:");
    println!("- MPU6050 readings:  {mpu6050_sample_rate} Hz (expected 100 Hz)");
    println!("- HMC8533L readings: {hmc8533l_sample_rate} Hz (expected 75 Hz)");

    Ok(())
}

fn determine_sampling<T: Time>(data: &[T]) -> (f32, f32) {
    let (total_diff, count) = data.windows(2).fold((0.0, 0), |(sum, cnt), window| {
        let diff = window[1].time() - window[0].time();
        (sum + diff, cnt + 1)
    });
    let sample_time = total_diff / (count as f32);
    let sample_rate = 1.0 / sample_time;
    (sample_rate, sample_time)
}

fn read_csv<T: DeserializeOwned>(file_path: &str) -> Result<Vec<T>, Box<dyn Error>> {
    let mut rdr = ReaderBuilder::new().from_path(file_path)?;
    let mut data = Vec::new();

    for result in rdr.deserialize() {
        let record: T = result?;
        data.push(record);
    }

    Ok(data)
}
