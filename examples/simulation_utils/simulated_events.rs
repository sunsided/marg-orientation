use crate::simulation_utils::{
    determine_sampling_rate, L3GD20Gyro, LSM303DLHCAccelerometer, LSM303DLHCMagnetometer, Time,
    Timed,
};
use csv::ReaderBuilder;
use marg_orientation::types::{AccelerometerReading, GyroscopeReading, MagnetometerReading};
use serde::de::DeserializeOwned;
use std::error::Error;
use std::iter::Peekable;
use std::slice::Iter;
use std::time::Duration;

pub struct SimulatedEvents {
    gyro: Vec<Timed<GyroscopeReading<f32>>>,
    compass: Vec<Timed<MagnetometerReading<f32>>>,
    accel: Vec<Timed<AccelerometerReading<f32>>>,
    simulation_time: Duration,
}

impl SimulatedEvents {
    pub fn new(path: &str) -> Result<SimulatedEvents, Box<dyn Error>> {
        let gyro =
            read_csv::<L3GD20Gyro>(format!("tests/data/{path}/106-gyro-i16-x1.csv").as_str())?;
        let compass = read_csv::<LSM303DLHCMagnetometer>(
            format!("tests/data/{path}/30-mag-i16-x3.csv").as_str(),
        )?;
        let accel = read_csv::<LSM303DLHCAccelerometer>(
            format!("tests/data/{path}/25-acc-i16-x3.csv").as_str(),
        )?;

        // Obtain the offset times.
        let gyro_t = gyro[0].time;
        let accel_t = accel[0].time;
        let compass_t = compass[0].time;
        let time_offset = gyro_t.min(accel_t).min(compass_t);

        // Convert the readings into normalized frames.
        let gyro: Vec<Timed<GyroscopeReading<f32>>> = L3GD20Gyro::vec_into_timed(gyro, time_offset);
        let compass: Vec<Timed<MagnetometerReading<f32>>> =
            LSM303DLHCMagnetometer::vec_into_timed(compass, time_offset);
        let accel: Vec<Timed<AccelerometerReading<f32>>> =
            LSM303DLHCAccelerometer::vec_into_timed(accel, time_offset);

        Ok(Self {
            gyro,
            compass,
            accel,
            simulation_time: Duration::default(),
        })
    }

    pub fn print_sampling_rates(&self) {
        print_sampling_rates(&self.gyro, &self.compass, &self.accel)
    }

    pub fn iter(&self) -> SimulatedEventIter {
        let count = self.gyro.len() + self.accel.len() + self.compass.len();
        SimulatedEventIter {
            count,
            gyro: self.gyro.iter().peekable(),
            mag: self.compass.iter().peekable(),
            accel: self.accel.iter().peekable(),
        }
    }
}

pub struct SimulatedEventIter<'a> {
    count: usize,
    gyro: Peekable<Iter<'a, Timed<GyroscopeReading<f32>>>>,
    mag: Peekable<Iter<'a, Timed<MagnetometerReading<f32>>>>,
    accel: Peekable<Iter<'a, Timed<AccelerometerReading<f32>>>>,
}

impl<'a> SimulatedEventIter<'a> {
    pub fn collect_until_into(
        iter: &mut Peekable<SimulatedEventIter>,
        simulation_time: Duration,
        events: &mut Vec<IterItem>,
    ) -> bool {
        let simulation_time = simulation_time.as_secs_f64();

        'collect: while let Some(event) = iter.peek() {
            if event.time() <= simulation_time {
                let event = iter.next().expect("item exists");
                events.push(event);
            } else {
                break 'collect;
            }
        }

        if iter.peek().is_none() && events.is_empty() {
            false
        } else {
            true
        }
    }
}

impl<'a> Iterator for SimulatedEventIter<'a> {
    type Item = IterItem;

    fn next(&mut self) -> Option<Self::Item> {
        let gyro_time = self.gyro.peek().map_or(f64::NEG_INFINITY, |d| d.time);
        let accel_time = self.accel.peek().map_or(f64::NEG_INFINITY, |d| d.time);
        let mag_time = self.mag.peek().map_or(f64::NEG_INFINITY, |d| d.time);
        if gyro_time < 0.0 && accel_time < 0.0 && mag_time < 0.0 {
            return None;
        }

        // Check if the gyro has the smallest time stamp.
        if gyro_time >= 0.0 && gyro_time <= accel_time && gyro_time <= mag_time {
            let gyro = self.gyro.next().cloned().expect("gyro has data");
            Some(IterItem::Gyro(gyro))
        }
        // Check if the gyro has the smallest time stamp.
        else if accel_time >= 0.0 && accel_time <= gyro_time && accel_time <= mag_time {
            let accel = self.accel.next().cloned().expect("accel has data");
            Some(IterItem::Accelerometer(accel))
        } else if mag_time >= 0.0 {
            debug_assert!(mag_time <= gyro_time && mag_time <= accel_time);
            let mag = self.mag.next().cloned().expect("mag has data");
            Some(IterItem::Magnetometer(mag))
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.count, Some(self.count))
    }
}

pub enum IterItem {
    Gyro(Timed<GyroscopeReading<f32>>),
    Accelerometer(Timed<AccelerometerReading<f32>>),
    Magnetometer(Timed<MagnetometerReading<f32>>),
}

impl IterItem {
    pub fn time(&self) -> f64 {
        match self {
            IterItem::Gyro(data) => data.time,
            IterItem::Accelerometer(data) => data.time,
            IterItem::Magnetometer(data) => data.time,
        }
    }
}

impl Time for IterItem {
    fn time(&self) -> f64 {
        self.time()
    }
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

fn print_sampling_rates(
    gyro: &Vec<Timed<GyroscopeReading<f32>>>,
    compass: &Vec<Timed<MagnetometerReading<f32>>>,
    accel: &Vec<Timed<AccelerometerReading<f32>>>,
) {
    let (gyro_sample_rate, _) = determine_sampling_rate(&gyro);
    let (accel_sample_rate, _) = determine_sampling_rate(&accel);
    let (compass_sample_rate, _) = determine_sampling_rate(&compass);

    println!("Average sample rates:");
    println!("- Accelerometer readings:  {accel_sample_rate} Hz (expected 400 Hz)");
    println!("- Magnetometer readings:  {compass_sample_rate} Hz (expected 75 Hz)");
    println!("- Gyroscope readings: {gyro_sample_rate} Hz (expected 400 Hz)");
}
