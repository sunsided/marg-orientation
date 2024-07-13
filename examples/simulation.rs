use coordinate_frame::{EastNorthUp, NorthEastDown, NorthWestDown, SouthEastUp, WestUpNorth};
use csv::ReaderBuilder;
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Rotation3, Scalar, Vector3};
use kiss3d::text::Font;
use kiss3d::window::Window;
use marg_orientation::gyro_free::{MagneticReference, OwnedOrientationEstimator};
use marg_orientation::{
    AccelerometerNoise, AccelerometerReading, GyroscopeReading, MagnetometerNoise,
    MagnetometerReading,
};
use serde::de::DeserializeOwned;
use serde::Deserialize;
use std::error::Error;
use std::ops::Deref;
use std::path::Path;
use std::time::{Duration, Instant};

const DISPLAY_REFERENCE: bool = true;
const DISPLAY_ESTIMATIONS: bool = true;
const DISPLAY_RAW_ACCEL: bool = true;
const DISPLAY_RAW_MAG: bool = true;

// const DATASET: &str = "serial-sensors/2024-07-10/stm32f3discovery/stationary";
const DATASET: &str = "serial-sensors/2024-07-10/stm32f3discovery/x-forward-rotate-around-up-ccw";
// const DATASET: &str = "serial-sensors/2024-07-10/stm32f3discovery/x-forward-tilt-top-east";

/// Kiss3d uses a West, Up, North system by default.
type Kiss3DCoordinates<T> = WestUpNorth<T>;

/// LSM303DLHC accelerometer readings.
#[derive(Debug, Deserialize)]
struct LSM303DLHCAccelerometer {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    time: f64,
    /// Accelerometer reading on the x-axis.
    #[serde(rename = "x")]
    acc_x: f32,
    /// Accelerometer reading on the y-axis.
    #[serde(rename = "y")]
    acc_y: f32,
    /// Accelerometer reading on the z-axis.
    #[serde(rename = "z")]
    acc_z: f32,
}

/// L3GD20 gyroscope readings.
#[derive(Debug, Deserialize)]
struct L3GD20Gyro {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    time: f64,
    /// Gyroscope reading on the x-axis.
    #[serde(rename = "x")]
    gyro_x: f32,
    /// Gyroscope reading on the y-axis.
    #[serde(rename = "y")]
    gyro_y: f32,
    /// Gyroscope reading on the z-axis.
    #[serde(rename = "z")]
    gyro_z: f32,
}

/// LSM303DLHC magnetometer readings.
#[derive(Debug, Deserialize)]
struct LSM303DLHCMagnetometer {
    /// The sample time, in seconds, relative to the Unix epoch.
    #[serde(rename = "host_time")]
    time: f64,
    /// Magnetometer reading on the x-axis.
    #[serde(rename = "x")]
    compass_x: f32,
    /// Magnetometer reading on the y-axis.
    #[serde(rename = "y")]
    compass_y: f32,
    /// Magnetometer reading on the z-axis.
    #[serde(rename = "z")]
    compass_z: f32,
}

pub trait Time {
    /// Gets the sample time.
    fn time(&self) -> f64;
}

impl Time for L3GD20Gyro {
    fn time(&self) -> f64 {
        self.time
    }
}

impl Time for LSM303DLHCAccelerometer {
    fn time(&self) -> f64 {
        self.time
    }
}

impl Time for LSM303DLHCMagnetometer {
    fn time(&self) -> f64 {
        self.time
    }
}

impl From<&LSM303DLHCAccelerometer> for AccelerometerReading<f32> {
    fn from(value: &LSM303DLHCAccelerometer) -> Self {
        // The HMC303DLHC's accelerometer on the STM32F3 Discovery board measures North, West, Down.
        let frame = NorthWestDown::new(value.acc_x, value.acc_y, value.acc_z);
        // Normalize by the sensor value range.
        let frame = frame / 16384.0;
        AccelerometerReading::north_east_down(frame)
    }
}

impl From<LSM303DLHCAccelerometer> for AccelerometerReading<f32> {
    fn from(value: LSM303DLHCAccelerometer) -> Self {
        Self::from(&value)
    }
}

impl From<&LSM303DLHCMagnetometer> for MagnetometerReading<f32> {
    fn from(value: &LSM303DLHCMagnetometer) -> Self {
        // The HMC303DLHC's magnetometer on the STM32F3 Discovery board measures South, East, Up.
        let frame = SouthEastUp::new(value.compass_x, value.compass_y, value.compass_z);
        // Normalize by the sensor value range.
        let frame = frame / 1100.0;
        MagnetometerReading::north_east_down(frame)
    }
}

impl From<LSM303DLHCMagnetometer> for MagnetometerReading<f32> {
    fn from(value: LSM303DLHCMagnetometer) -> Self {
        Self::from(&value)
    }
}

impl From<&L3GD20Gyro> for GyroscopeReading<f32> {
    fn from(value: &L3GD20Gyro) -> Self {
        // The L3GD20 gyroscope on the STM32F3 Discovery board measures East, North, Up.
        let frame = EastNorthUp::new(value.gyro_x, value.gyro_y, value.gyro_z);
        // Normalize by the sensor value range.
        let frame = frame / 5.714285;
        GyroscopeReading::north_east_down(frame)
    }
}

impl From<L3GD20Gyro> for GyroscopeReading<f32> {
    fn from(value: L3GD20Gyro) -> Self {
        Self::from(&value)
    }
}

/// A timed reading.
pub struct Timed<R> {
    pub time: f64,
    pub reading: R,
}

impl<T> Timed<T> {
    pub fn with_time_offset(mut self, value: f64) -> Self {
        self.time -= value;
        self
    }
}

impl<R> Time for Timed<R> {
    fn time(&self) -> f64 {
        self.time
    }
}

impl From<LSM303DLHCAccelerometer> for Timed<AccelerometerReading<f32>> {
    fn from(value: LSM303DLHCAccelerometer) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}

impl From<LSM303DLHCMagnetometer> for Timed<MagnetometerReading<f32>> {
    fn from(value: LSM303DLHCMagnetometer) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}

impl From<L3GD20Gyro> for Timed<GyroscopeReading<f32>> {
    fn from(value: L3GD20Gyro) -> Self {
        Self {
            time: value.time,
            reading: value.into(),
        }
    }
}

impl<R> Deref for Timed<R> {
    type Target = R;

    fn deref(&self) -> &Self::Target {
        &self.reading
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let gyro =
        read_csv::<L3GD20Gyro>(format!("tests/data/{DATASET}/106-gyro-i16-x1.csv").as_str())?;
    let compass = read_csv::<LSM303DLHCMagnetometer>(
        format!("tests/data/{DATASET}/30-mag-i16-x3.csv").as_str(),
    )?;
    let accel = read_csv::<LSM303DLHCAccelerometer>(
        format!("tests/data/{DATASET}/25-acc-i16-x3.csv").as_str(),
    )?;

    // Obtain the offset times.
    let gyro_t = gyro[0].time;
    let accel_t = accel[0].time;
    let compass_t = compass[0].time;
    let time_offset = gyro_t.min(accel_t).min(compass_t);

    // Convert the readings into normalized frames.
    let gyro: Vec<Timed<GyroscopeReading<f32>>> = gyro
        .into_iter()
        .map(Timed::from)
        .map(|t| t.with_time_offset(time_offset))
        .collect();
    let compass: Vec<Timed<MagnetometerReading<f32>>> = compass
        .into_iter()
        .map(Timed::from)
        .map(|t| t.with_time_offset(time_offset))
        .collect();
    let accel: Vec<Timed<AccelerometerReading<f32>>> = accel
        .into_iter()
        .map(Timed::from)
        .map(|t| t.with_time_offset(time_offset))
        .collect();

    // Determine sample rates.
    let (gyro_sample_rate, _) = determine_sampling(&gyro);
    let (accel_sample_rate, _) = determine_sampling(&accel);
    let (compass_sample_rate, _) = determine_sampling(&compass);

    println!("Average sample rates:");
    println!("- Accelerometer readings:  {accel_sample_rate} Hz (expected 400 Hz)");
    println!("- Magnetometer readings:  {compass_sample_rate} Hz (expected 75 Hz)");
    println!("- Gyroscope readings: {gyro_sample_rate} Hz (expected 400 Hz)");

    // Magnetic field reference for Berlin, Germany expressed in North, East, Down.
    let reference = MagneticReference::new(18.0, 1.5, 47.0);

    // Create the estimator.
    let mut estimator = OwnedOrientationEstimator::<f32>::new(
        AccelerometerNoise::new(0.07, 0.07, 0.07),
        MagnetometerNoise::new(0.18, 0.11, 0.34),
        reference,
        0.001,
        1e-6,
    );

    let mut gyro_x_estimator =
        marg_orientation::gyro_drift::GyroscopeAxisEstimator::<f32>::new(0.00003, 5.0, 0.001);

    // Prepare some basics for the simulation.
    let font = Font::new(Path::new(
        "examples/data/fonts/firamono/FiraMono-Regular.ttf",
    ))
    .expect("failed to load font");

    let mut window = Window::new("MPU6050 and HMC8533L simulation");
    window.set_framerate_limit(Some(30));
    window.set_background_color(0.2, 0.2, 0.2);

    let mut c = window.add_cube(0.02, 0.02, 0.02);
    c.set_color(1.0, 1.0, 1.0);

    window.set_light(Light::StickToCamera);

    // Some colors.
    let red = Point3::new(1.0, 0.0, 0.0);
    let green = Point3::new(0.0, 1.0, 0.0);
    let blue = Point3::new(0.0, 0.0, 1.0);
    let dark_red = red * 0.5;
    let dark_green = green * 0.5;
    let dark_blue = blue * 0.5;

    // Walk through the simulation data.
    let mut accel_index = 0;
    let mut compass_index = 0;
    let mut gyro_index = 0;

    let mut last_time = Instant::now();
    let mut simulation_time = Duration::default();
    while window.render() {
        // Obtain the current render timestamp.
        let now = Instant::now();
        let elapsed_time = now - last_time;
        simulation_time += elapsed_time;
        last_time = now;

        // Enable updates when we receive new data.
        let mut acc_should_update = false;
        let mut mag_should_update = false;
        let mut gyro_should_update = false;

        // Increment simulation index.
        while accel[accel_index].time < simulation_time.as_secs_f64() {
            accel_index += 1;
            acc_should_update = true;
            if accel_index >= gyro.len() {
                accel_index = 0;
                gyro_index = 0;
                compass_index = 0;
                simulation_time = Duration::default();
            }
        }
        while gyro[gyro_index].time < simulation_time.as_secs_f64() {
            gyro_index += 1;
            gyro_should_update = true;
            if accel_index >= gyro.len() {
                accel_index = 0;
                gyro_index = 0;
                compass_index = 0;
                simulation_time = Duration::default();
            }
        }
        while compass[compass_index].time < simulation_time.as_secs_f64() {
            compass_index += 1;
            mag_should_update = true;
            if compass_index >= compass.len() {
                accel_index = 0;
                gyro_index = 0;
                compass_index = 0;
                simulation_time = Duration::default();
            }
        }

        let accel_meas = &accel[accel_index];
        let gyro_meas = &gyro[accel_index];
        let compass_meas = &compass[compass_index];

        // Calculate the angle between the magnetic field vector and the down vector.
        let angle_mag_accel = calculate_angle_acc_mag(accel_meas, compass_meas);

        // Calculated heading.
        let heading_degrees = {
            let vec = Vector3::new(compass_meas.x, compass_meas.y, compass_meas.z).normalize();
            let heading = vec[1].atan2(vec[0]).to_degrees();
            if heading >= 360.0 {
                heading - 360.0
            } else if heading <= 0.0 {
                heading + 360.0
            } else {
                heading
            }
        };

        // Run a prediction.
        estimator.predict();
        gyro_x_estimator.predict(elapsed_time.as_secs_f32());

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!("nan before correction");
        }

        // Update the filter when needed.
        if acc_should_update {
            estimator.correct_accelerometer(&accel_meas.reading);
        }
        if mag_should_update {
            estimator.correct_magnetometer(&compass_meas.reading);
        }
        if gyro_should_update {
            gyro_x_estimator.correct(gyro_meas.reading.omega_x);
        }

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!("nan after correction");
        }

        // Obtain a rotation matrix from the estimated angles.
        let estimated_angles = estimator.estimated_angles();
        /*
        let rotation = Rotation3::from_euler_angles(
            estimated_angles.roll_phi,
            estimated_angles.pitch_theta,
            estimated_angles.yaw_psi,
        );

        let north = NorthEastDown::new(1.0, 0.0, 0.0);
        let east = NorthEastDown::new(0.0, 1.0, 0.0);
        let down = NorthEastDown::new(0.0, 0.0, 1.0);

        let north = Point3::new(north.x(), north.y(), north.z());
        let east = Point3::new(east.x(), east.y(), east.z());
        let down = Point3::new(down.x(), down.y(), down.z());

        let filter_x = Point3::from(rotation * north);
        let filter_y = Point3::from(rotation * east);
        let filter_z = Point3::from(rotation * down);
        */

        let north = estimator.rotate_vector(marg_orientation::Vector3::new(1.0, 0.0, 0.0));
        let east = estimator.rotate_vector(marg_orientation::Vector3::new(0.0, 1.0, 0.0));
        let down = estimator.rotate_vector(marg_orientation::Vector3::new(0.0, 0.0, 1.0));

        let filter_x = kiss3d_point(NorthEastDown::new(north.x, north.y, north.z));
        let filter_y = kiss3d_point(NorthEastDown::new(east.x, east.y, east.z));
        let filter_z = kiss3d_point(NorthEastDown::new(down.x, down.y, down.z));

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
        let info = format!("tm = {:.2} s (#{})", gyro[accel_index].time, accel_index);
        window.draw_text(
            &info,
            &Point2::new(0.0, 64.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display default coordinate system.
        #[allow(dead_code)]
        if DISPLAY_REFERENCE {
            let x_axis = NorthEastDown::new(1.0, 0.0, 0.0);
            let y_axis = NorthEastDown::new(0.0, 1.0, 0.0);
            let z_axis = NorthEastDown::new(0.0, 0.0, 1.0);

            window.draw_line(&Point3::default(), &kiss3d_point(x_axis), &dark_red);
            window.draw_line(&Point3::default(), &kiss3d_point(y_axis), &dark_green);
            window.draw_line(&Point3::default(), &kiss3d_point(z_axis), &dark_blue);
        }

        // Convert estimations.
        #[allow(dead_code)]
        if DISPLAY_ESTIMATIONS {
            let ex = NorthEastDown::new(filter_x[0], filter_x[1], filter_x[2]);
            let ey = NorthEastDown::new(filter_y[0], filter_y[1], filter_y[2]);
            let ez = NorthEastDown::new(filter_z[0], filter_z[1], filter_z[2]);

            // Display estimated orientation.
            window.draw_line(&Point3::default(), &filter_x, &red);
            window.draw_line(&Point3::default(), &filter_y, &green);
            window.draw_line(&Point3::default(), &filter_z, &blue);
        }

        // Display the accelerometer reading.
        let am = NorthEastDown::new(accel_meas.x, accel_meas.y, accel_meas.z);
        #[allow(dead_code)]
        if DISPLAY_RAW_ACCEL {
            let p1 = Point3::new(0.0, 0.0, 0.0);
            window.draw_line(&p1, &kiss3d_point(am), &Point3::new(0.5, 0.0, 1.0));
        }

        // Display the compass reading.
        let mm = NorthEastDown::new(compass_meas.x, compass_meas.y, compass_meas.z);
        #[allow(dead_code)]
        if DISPLAY_RAW_MAG {
            let p1 = Point3::new(0.0, 0.0, 0.0);
            window.draw_line(&p1, &kiss3d_point(mm), &Point3::new(1.0, 0.0, 0.5));
        }

        // Display simulation indexes.
        let info = format!(
            "th = {:.2} s (#{})",
            compass[compass_index].time, compass_index
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 92.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display angle between measured accelerometer and magnetometer.
        let info = format!("cos⁻¹(acc·mag) = {:+0.02}°", angle_mag_accel.to_degrees());
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 18.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display angle between measured accelerometer and magnetometer.
        let info = format!("heading = {:+0.02}°", heading_degrees);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 17.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display magnetometer.
        let info = format!("Bx = {:+0.02} Gs", compass_meas.x);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 15.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display magnetometer.
        let info = format!("By = {:+0.02} Gs", compass_meas.y);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 14.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display magnetometer.
        let info = format!("Bz = {:+0.02} Gs", compass_meas.z);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 13.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display acceleration.
        let info = format!("ax = {:+0.02} G", accel_meas.x);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 11.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display acceleration.
        let info = format!("ay = {:+0.02} G", accel_meas.y);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 10.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display acceleration.
        let info = format!("az = {:+0.02} G", accel_meas.z);
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 9.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display gyro roll rates.
        let estimated_omega_x = gyro_x_estimator.angular_velocity();
        let estimated_x_bias = gyro_x_estimator.bias();
        let info = format!(
            "ωx = {:+0.02} rad/s ({:+0.02}°/s) - {:+0.02} rad/s ± {:+0.02}rad/s",
            gyro_meas.omega_x,
            gyro_meas.omega_x * 180.0 / std::f32::consts::PI,
            estimated_omega_x,
            estimated_x_bias
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 7.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display gyro roll rates.
        let info = format!(
            "ωy = {:+0.02} rad/s ({:+0.02}°/s)",
            gyro_meas.omega_y,
            gyro_meas.omega_y * 180.0 / std::f32::consts::PI
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 6.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display gyro roll rates.
        let info = format!(
            "ωz = {:+0.02} rad/s ({:+0.02}°/s)",
            gyro_meas.omega_z,
            gyro_meas.omega_z * 180.0 / std::f32::consts::PI
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0 * 5.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Display estimated angles.
        let info = format!(
            "φ = {:+0.02} ± {:+0.02} rad ({:+0.02}°)",
            estimated_angles.roll_phi,
            estimator.roll_variance().sqrt(),
            estimated_angles.roll_phi * 180.0 / std::f32::consts::PI,
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
            estimated_angles.pitch_theta * 180.0 / std::f32::consts::PI,
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
            estimated_angles.yaw_psi * 180.0 / std::f32::consts::PI,
        );
        window.draw_text(
            &info,
            &Point2::new(0.0, 1200.0 - 32.0),
            32.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );
    }

    Ok(())
}

fn calculate_angle_acc_mag(
    accel_meas: &Timed<AccelerometerReading<f32>>,
    compass_meas: &Timed<MagnetometerReading<f32>>,
) -> f32 {
    let accel_vec: Vector3<_> = Vector3::new(accel_meas.x, accel_meas.y, accel_meas.z).normalize();
    let mag_vec: Vector3<_> =
        Vector3::new(compass_meas.x, compass_meas.y, compass_meas.z).normalize();
    accel_vec.dot(&mag_vec).acos()
}

fn determine_sampling<T: Time>(data: &[T]) -> (f64, f64) {
    let (total_diff, count) = data.windows(2).fold((0.0, 0), |(sum, cnt), window| {
        let diff = window[1].time() - window[0].time();
        (sum + diff, cnt + 1)
    });
    let sample_time = total_diff / (count as f64);
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

fn kiss3d_point<C, T>(vector: C) -> Point3<T>
where
    C: Into<Kiss3DCoordinates<T>>,
    T: Scalar,
{
    let vector = vector.into();
    Point3::new(vector.x(), vector.y(), vector.z())
}
