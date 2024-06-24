use csv::ReaderBuilder;
use serde::de::DeserializeOwned;
use serde::Deserialize;
use std::error::Error;
use std::path::Path;
use std::time::{Duration, Instant};

use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Rotation3, Vector3};
use kiss3d::text::Font;
use kiss3d::window::Window;
use marg_orientation::{
    AccelerometerNoise, AccelerometerReading, GyroscopeNoise, GyroscopeReading, MagnetometerNoise,
    MagnetometerReading, OwnedOrientationEstimator,
};

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

impl From<&MPU6050> for AccelerometerReading<f32> {
    fn from(value: &MPU6050) -> Self {
        AccelerometerReading::new(value.acc_x, value.acc_y, value.acc_z)
    }
}

impl From<&MPU6050> for GyroscopeReading<f32> {
    fn from(value: &MPU6050) -> Self {
        GyroscopeReading::new(value.gyro_x, value.gyro_y, value.gyro_z)
    }
}

impl From<&HMC5833L> for MagnetometerReading<f32> {
    fn from(value: &HMC5833L) -> Self {
        MagnetometerReading::new(value.compass_x, value.compass_y, value.compass_z)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let dataset = "set-2/unmoved-with-x-pointing-forward";

    let mpu6050 = read_csv::<MPU6050>(
        format!("tests/data/sensor-fusion/data/{dataset}/mpu6050.csv").as_str(),
    )?;
    let hmc5833l = read_csv::<HMC5833L>(
        format!("tests/data/sensor-fusion/data/{dataset}/hmc5833l.csv").as_str(),
    )?;

    // println!("{:#?}", mpu6050[0]);
    // println!("{:#?}", hmc5833l[0]);

    // Determine sample rates.
    let (mpu6050_sample_rate, _) = determine_sampling(&mpu6050);
    let (hmc8533l_sample_rate, _) = determine_sampling(&hmc5833l);

    println!("Average sample rates:");
    println!("- MPU6050 readings:  {mpu6050_sample_rate} Hz (expected 100 Hz)");
    println!("- HMC8533L readings: {hmc8533l_sample_rate} Hz (expected 75 Hz)");

    // Create the estimator.
    let mut estimator = OwnedOrientationEstimator::<f32>::new(
        0.01,
        AccelerometerNoise::new(0.07, 0.07, 0.07),
        GyroscopeNoise::new(0.54, 0.1, 0.13),
        MagnetometerNoise::new(0.18, 0.11, 0.34),
        1e-6,
    );

    // Prepare some basics for the simulation.
    let font = Font::new(Path::new(
        "examples/data/fonts/firamono/FiraMono-Regular.ttf",
    ))
    .expect("failed to load font");

    let mut window = Window::new("MPU6050 and HMC8533L simulation");
    window.set_framerate_limit(Some(30));

    let mut c = window.add_cube(0.02, 0.02, 0.02);
    c.set_color(1.0, 1.0, 1.0);

    window.set_light(Light::StickToCamera);

    let mut mpu6050_index = 0;
    let mut hmc8583l_index = 0;

    let mut last_time = Instant::now();
    let mut simulation_time = Duration::default();
    while window.render() {
        // Obtain the current render timestamp.
        let now = Instant::now();
        let elapsed_time = now - last_time;
        simulation_time += elapsed_time;
        last_time = now;

        // Enable updates when we receive new data.
        let mut should_update = false;

        // Increment simulation index.
        while mpu6050[mpu6050_index].time < simulation_time.as_secs_f32() {
            mpu6050_index += 1;
            should_update = true;
            if mpu6050_index >= mpu6050.len() {
                mpu6050_index = 0;
                hmc8583l_index = 0;
                simulation_time = Duration::default();
            }
        }
        while hmc5833l[hmc8583l_index].time < simulation_time.as_secs_f32() {
            hmc8583l_index += 1;
            should_update = true;
            if hmc8583l_index >= hmc5833l.len() {
                mpu6050_index = 0;
                hmc8583l_index = 0;
                simulation_time = Duration::default();
            }
        }

        let mpu6050_meas = &mpu6050[mpu6050_index];
        let hmc5833l_meas = &hmc5833l[hmc8583l_index];

        // Run a prediction.
        estimator.predict(
            elapsed_time.as_secs_f32(),
            &GyroscopeReading::from(mpu6050_meas),
        );

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!();
        }

        // Update the filter when needed.
        if should_update {
            let accelerometer = AccelerometerReading::from(mpu6050_meas);
            let magnetometer = MagnetometerReading::from(hmc5833l_meas);
            estimator.correct(&accelerometer, &magnetometer);
        }

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!();
        }

        // Obtain a rotation matrix from the estimated angles.
        let estimated_angles = estimator.estimated_angles();
        let rotation = Rotation3::from_euler_angles(
            estimated_angles.roll_phi,
            estimated_angles.pitch_theta,
            estimated_angles.yaw_psi,
        );
        let filter_x = Point3::from((rotation * Vector3::new(1.0, 0.0, 0.0)).normalize());
        let filter_y = Point3::from((rotation * Vector3::new(0.0, 1.0, 0.0)).normalize());
        let filter_z = Point3::from((rotation * Vector3::new(0.0, 0.0, 1.0)).normalize());

        // Display elapsed time since last frame.
        let info = format!(
            "ΔT = {:.4} s ({:.2}) Hz",
            elapsed_time.as_secs_f32(),
            elapsed_time.as_secs_f32().recip()
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 0.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display simulation time.
        let info = format!(" t = {:.2} s", simulation_time.as_secs_f32());
        window.draw_text(
            &info,
            &Point2::new(0.0, 32.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display simulation indexes.
        let info = format!(
            "tm = {:.2} s (#{})",
            mpu6050[mpu6050_index].time, mpu6050_index
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 64.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display default coordinate system.
        window.draw_line(
            &Point3::default(),
            &Point3::new(1.0, 0.0, 0.0),
            &Point3::new(1.0, 0.0, 0.0),
        );
        window.draw_line(
            &Point3::default(),
            &Point3::new(0.0, 1.0, 0.0),
            &Point3::new(0.0, 1.0, 0.0),
        );
        window.draw_line(
            &Point3::default(),
            &Point3::new(0.0, 0.0, 1.0),
            &Point3::new(0.0, 0.0, 1.0),
        );

        // Display estimated orientation.
        window.draw_line(&Point3::default(), &filter_x, &Point3::new(1.0, 1.0, 1.0));
        window.draw_line(&Point3::default(), &filter_y, &Point3::new(1.0, 1.0, 1.0));
        window.draw_line(&Point3::default(), &filter_z, &Point3::new(1.0, 1.0, 1.0));

        // Display simulation indexes.
        let info = format!(
            "th = {:.2} s (#{})",
            hmc5833l[hmc8583l_index].time, hmc8583l_index
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 92.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display estimated angles.
        let info = format!(
            "φ = {:+0.02} ± {:+0.02} rad ({:+0.02}°)",
            estimated_angles.roll_phi,
            estimator.roll_variance().sqrt(),
            estimated_angles.roll_phi * 180.0 / std::f32::consts::PI
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 3.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display estimated angles.
        let info = format!(
            "θ = {:+0.02} ± {:+0.02} rad ({:+0.02}°)",
            estimated_angles.pitch_theta,
            estimator.pitch_variance().sqrt(),
            estimated_angles.pitch_theta * 180.0 / std::f32::consts::PI
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 2.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display estimated angles.
        let info = format!(
            "ψ = {:+0.02} ± {:+0.02} rad ({:+0.02}°)",
            estimated_angles.yaw_psi,
            estimator.yaw_variance().sqrt(),
            estimated_angles.yaw_psi * 180.0 / std::f32::consts::PI
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display the accelerometer reading.
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(mpu6050_meas.acc_x, mpu6050_meas.acc_y, mpu6050_meas.acc_z);
        window.draw_line(&p1, &p2, &Point3::new(1.0, 1.0, 0.0));

        // Display the compass reading.
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(
            hmc5833l_meas.compass_x,
            hmc5833l_meas.compass_y,
            hmc5833l_meas.compass_z,
        );
        window.draw_line(&p1, &p2, &Point3::new(1.0, 0.0, 1.0));
    }

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
