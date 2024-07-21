use std::cell::RefCell;
use std::error::Error;
use std::path::Path;
use std::rc::Rc;
use std::time::{Duration, Instant};

use coordinate_frame::NorthEastDown;
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Matrix3, Point2, Point3, Scalar, Translation3, UnitQuaternion, Vector3};
use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;

use marg_orientation::dcm::OwnedOrientationEstimator;
use marg_orientation::types::{
    AccelerometerNoise, AccelerometerReading, GyroscopeBias, GyroscopeNoise, GyroscopeReading,
    MagnetometerNoise, MagnetometerReading,
};

use crate::simulation_utils::{
    IterItem, Kiss3DCoordinates, SimulatedEventIter, SimulatedEvents, Timed,
};

mod simulation_utils;

// const DATASET: &str = "2024-07-10/stm32f3discovery/stationary";
const DATASET: &str = "2024-07-10/stm32f3discovery/x-forward-rotate-around-up-ccw";
// const DATASET: &str = "2024-07-10/stm32f3discovery/x-forward-tilt-top-east";
// const DATASET: &str = "2024-07-10/stm32f3discovery/x-forward-tilt-nose-up";
// const DATASET: &str = "2024-07-06/stm32f3discovery";

fn main() -> Result<(), Box<dyn Error>> {
    let simulated_events = SimulatedEvents::new(DATASET)?;

    // Determine sample rates.
    simulated_events.print_sampling_rates();

    // Create the estimator.
    let mut estimator = OwnedOrientationEstimator::<f32>::new(
        AccelerometerNoise::new(0.07, 0.07, 0.07),
        MagnetometerNoise::new(0.18, 0.11, 0.34),
        GyroscopeNoise::new(0.02, 0.02, 0.02),
        0.1,
        1.0, // 1
        1.0, // 1
        1.0, // 10
        1.0, // 10
    );

    let mut gyro_estimator = marg_orientation::GyroRateAndDriftEstimator::<f32>::new(
        GyroscopeBias::new(0.00003, 0.00003, 0.00003),
        GyroscopeNoise::new(0.05, 0.05, 0.05),
        0.001,
    );

    // Prepare some basics for the simulation.
    let font = Font::new(Path::new(
        "examples/data/fonts/firamono/FiraMono-Regular.ttf",
    ))
    .expect("failed to load font");

    let mut window = Window::new("LSM303DLHC and L3GD20 simulation");
    window.set_framerate_limit(Some(30));
    window.set_light(Light::StickToCamera);

    // Create a box at the coordinate origin.
    let mut c = window.add_cube(0.02, 0.02, 0.02);
    c.set_color(1.0, 1.0, 1.0);

    // Create the arrow indicating orientation.
    let mut arrows = create_arrow(&mut window);

    // Some colors.
    let red = Point3::new(1.0, 0.0, 0.0);
    let green = Point3::new(0.0, 1.0, 0.0);
    let blue = Point3::new(0.0, 0.0, 1.0);
    let dark_red = red * 0.5;
    let dark_green = green * 0.5;
    let dark_blue = blue * 0.5;

    // Walk through the simulation data.
    let mut event_iter = simulated_events.iter().peekable();
    let mut events = Vec::new();
    let mut accel_meas = Timed {
        time: 0.0,
        reading: AccelerometerReading::new(f32::NAN, f32::NAN, f32::NAN),
    };
    let mut gyro_meas = Timed {
        time: 0.0,
        reading: GyroscopeReading::new(f32::NAN, f32::NAN, f32::NAN),
    };
    let mut compass_meas = Timed {
        time: 0.0,
        reading: MagnetometerReading::new(f32::NAN, f32::NAN, f32::NAN),
    };
    let mut last_event_time = 0.0;

    let mut last_time = Instant::now();
    let mut simulation_time = Duration::default();
    let mut is_paused = false;
    let mut reset_times = true;
    let mut display_reference = true;
    let mut display_body_frame = false;
    let mut display_sensors = true;

    'render: while window.render() {
        // Obtain the current render timestamp.
        let now = Instant::now();
        let elapsed_time = now - last_time;
        last_time = now;

        if reset_times {
            reset_times = false;
            accel_meas = Timed {
                time: 0.0,
                reading: AccelerometerReading::new(0.0, 0.0, 1.0),
            };
            gyro_meas = Timed {
                time: 0.0,
                reading: GyroscopeReading::default(),
            };
            compass_meas = Timed {
                time: 0.0,
                reading: MagnetometerReading::new(1.0, 0.0, 0.0),
            };
            simulation_time = Duration::default();
            last_event_time = 0.0;
            event_iter = simulated_events.iter().peekable();
        }

        // Handle window events to check for key presses
        for event in window.events().iter() {
            if let WindowEvent::Key(key, Action::Press, _) = event.value {
                if key == Key::Space {
                    is_paused = !is_paused;
                } else if key == Key::R {
                    reset_times = true;
                    continue;
                } else if key == Key::C {
                    display_reference = !display_reference;
                    continue;
                } else if key == Key::B {
                    display_body_frame = !display_body_frame;
                    continue;
                } else if key == Key::S {
                    display_sensors = !display_sensors;
                    continue;
                }
            }
        }

        if !is_paused {
            simulation_time += elapsed_time;
            window.set_background_color(0.118, 0.122, 0.149);
        } else {
            window.set_background_color(0.149, 0.122, 0.118);
        }

        // Increment simulation index.
        if !is_paused {
            events.clear();

            if !SimulatedEventIter::collect_until_into(
                &mut event_iter,
                simulation_time,
                &mut events,
            ) {
                reset_times = true;
                continue 'render;
            }
        }

        // Apply all updates we missed in between rendering frames.
        for event in events.drain(..) {
            let mut elapsed_time = (event.time() - last_event_time) as f32;
            if elapsed_time == 0.0 {
                elapsed_time = 0.01;
            }

            last_event_time = event.time();

            // Update the estimator.
            estimator.update(elapsed_time);

            match event {
                IterItem::Gyro(data) => {
                    gyro_meas = data;
                    gyro_estimator.correct(gyro_meas.reading);
                }
                IterItem::Accelerometer(data) => {
                    accel_meas = data;
                    estimator.correct_accelerometer(
                        accel_meas.reading,
                        gyro_meas.reading,
                        elapsed_time,
                    );
                }
                IterItem::Magnetometer(data) => {
                    compass_meas = data;
                    estimator.correct_magnetometer(
                        compass_meas.reading,
                        gyro_meas.reading,
                        elapsed_time,
                    );
                }
            }
        }

        // Calculate the angle between the magnetic field vector and the down vector.
        let angle_mag_accel = calculate_angle_acc_mag(&accel_meas, &compass_meas);

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

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!("nan before correction");
        }

        let estimated_angles = estimator.estimated_angles();
        if estimated_angles.yaw_psi.is_nan()
            || estimated_angles.pitch_theta.is_nan()
            || estimated_angles.roll_phi.is_nan()
        {
            todo!("nan after correction");
        }

        // Obtain a rotation matrix from the estimated angles.
        let north =
            estimator.rotate_vector_world(marg_orientation::types::Vector3::new(1.0, 0.0, 0.0));
        let east =
            estimator.rotate_vector_world(marg_orientation::types::Vector3::new(0.0, 1.0, 0.0));
        let down =
            estimator.rotate_vector_world(marg_orientation::types::Vector3::new(0.0, 0.0, 1.0));
        let filter_x = kiss3d_point(NorthEastDown::new(north.x, north.y, north.z));
        let filter_y = kiss3d_point(NorthEastDown::new(east.x, east.y, east.z));
        let filter_z = kiss3d_point(NorthEastDown::new(down.x, down.y, down.z));

        // Update the arrow according to the estimations.
        update_arrow_orientation(&mut arrows, filter_x, filter_y, filter_z);

        // Display default coordinate system.
        if display_reference {
            let x_axis = NorthEastDown::new(1.0, 0.0, 0.0);
            let y_axis = NorthEastDown::new(0.0, 1.0, 0.0);
            let z_axis = NorthEastDown::new(0.0, 0.0, 1.0);

            window.draw_line(&Point3::default(), &kiss3d_point(x_axis), &dark_red);
            window.draw_line(&Point3::default(), &kiss3d_point(y_axis), &dark_green);
            window.draw_line(&Point3::default(), &kiss3d_point(z_axis), &dark_blue);
        }

        // Convert estimations.
        if display_body_frame {
            // Display estimated orientation.
            window.draw_line(&Point3::default(), &filter_x, &red);
            window.draw_line(&Point3::default(), &filter_y, &green);
            window.draw_line(&Point3::default(), &filter_z, &blue);
        }

        // Display the accelerometer reading.
        if display_sensors {
            let am = NorthEastDown::new(accel_meas.x, accel_meas.y, accel_meas.z);
            let p1 = Point3::new(0.0, 0.0, 0.0);
            window.draw_line(&p1, &kiss3d_point(am), &Point3::new(0.5, 0.0, 1.0));
        }

        // Display the compass reading.
        if display_sensors {
            let mm = NorthEastDown::new(compass_meas.x, compass_meas.y, compass_meas.z);
            let p1 = Point3::new(0.0, 0.0, 0.0);
            window.draw_line(&p1, &kiss3d_point(mm), &Point3::new(1.0, 0.0, 0.5));
        }

        display_times_and_indexes(
            &gyro_meas,
            &compass_meas,
            &accel_meas,
            &font,
            &mut window,
            simulation_time,
            elapsed_time,
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
        let estimated_omega = gyro_estimator.rate_estimate();
        let estimated_bias = gyro_estimator.bias_estimate();
        let info = format!(
            "ωx = {:+0.02} rad/s ({:+0.02}°/s) - {:+0.02} rad/s ± {:+0.02}rad/s",
            gyro_meas.omega_x,
            gyro_meas.omega_x * 180.0 / std::f32::consts::PI,
            estimated_omega.omega_x,
            estimated_bias.omega_x
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
            "φ = {:+0.02} ({:+0.02}°)",
            estimated_angles.roll_phi,
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
            "θ = {:+0.02} ({:+0.02}°)",
            estimated_angles.pitch_theta,
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
            "ψ = {:+0.02} ({:+0.02}°)",
            estimated_angles.yaw_psi,
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

#[allow(clippy::too_many_arguments)]
fn display_times_and_indexes(
    gyro: &Timed<GyroscopeReading<f32>>,
    compass: &Timed<MagnetometerReading<f32>>,
    accel: &Timed<AccelerometerReading<f32>>,
    font: &Rc<Font>,
    window: &mut Window,
    simulation_time: Duration,
    elapsed_time: Duration,
) {
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
        font,
        &Point3::new(1.0, 1.0, 1.0),
    );

    // Display simulation time.
    let info = format!(" t = {:.2} s", simulation_time.as_secs_f32());
    window.draw_text(
        &info,
        &Point2::new(0.0, 32.0),
        32.0,
        font,
        &Point3::new(1.0, 1.0, 1.0),
    );

    // Display simulation indexes.
    let info = format!("ta = {:.2} s", accel.time);
    window.draw_text(
        &info,
        &Point2::new(0.0, 32.0 + 32.0),
        32.0,
        font,
        &Point3::new(1.0, 1.0, 1.0),
    );

    // Display simulation indexes.
    let info = format!("tm = {:.2} s", compass.time);
    window.draw_text(
        &info,
        &Point2::new(0.0, 32.0 + 2.0 * 32.0),
        32.0,
        font,
        &Point3::new(1.0, 1.0, 1.0),
    );

    // Display simulation indexes.
    let info = format!("tg = {:.2} s", gyro.time);
    window.draw_text(
        &info,
        &Point2::new(0.0, 32.0 + 3.0 * 32.0),
        32.0,
        font,
        &Point3::new(1.0, 1.0, 1.0),
    );
}

fn create_arrow(window: &mut Window) -> SceneNode {
    // Define the vertices for the arrow, based on a rectangle (shaft) and triangles (head).
    let arrow_vertices = vec![
        // Rectangle vertices (shaft) on x-z plane, half as long
        Point3::new(-0.20, 0.0, -0.05), // Bottom-left
        Point3::new(0.10, 0.0, -0.05),  // Bottom-right
        Point3::new(0.10, 0.0, 0.05),   // Top-right
        Point3::new(-0.20, 0.0, 0.05),  // Top-left
        // Triangle vertices (head) on x-z plane, half as long
        Point3::new(0.10, 0.0, -0.1), // Bottom
        Point3::new(0.30, 0.0, 0.0),  // Tip
        Point3::new(0.10, 0.0, 0.1),  // Top
    ];

    // Define the indices for the arrow.
    let arrow_indices = vec![
        // Rectangle (shaft)
        Point3::new(0u16, 1, 2),
        Point3::new(0, 2, 3),
        // Triangle (head)
        Point3::new(4, 5, 6),
    ];

    // Create the mesh from vertices and indices
    let arrow_mesh = Mesh::new(arrow_vertices, arrow_indices, None, None, false);
    let arrow_mesh = Rc::new(RefCell::new(arrow_mesh));

    // Quaternion to flip the arrow around once.
    let flip_quaternion = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI);

    // Add the mesh to the window
    let mut arrows = window.add_group();

    // The top-side of the arrow (red).
    let mut top_arrow = arrows.add_mesh(arrow_mesh.clone(), Vector3::new(1.0, 1.0, 1.0));
    top_arrow.set_color(0.8, 0.165, 0.212);
    top_arrow.set_local_translation(Translation3::new(0.0, -0.005, 0.0));

    // Flipped version to work around backface culling.
    let mut top_arrow = arrows.add_mesh(arrow_mesh.clone(), Vector3::new(1.0, 1.0, 1.0));
    top_arrow.set_color(0.545, 0.114, 0.145);
    top_arrow.set_local_translation(Translation3::new(0.0, -0.005, 0.0));
    top_arrow.set_local_rotation(flip_quaternion);

    // The bottom-side of the arrow (white).
    let mut bottom_arrow = arrows.add_mesh(arrow_mesh.clone(), Vector3::new(1.0, 1.0, 1.0));
    bottom_arrow.set_color(1.0, 1.0, 1.0);
    bottom_arrow.set_local_translation(Translation3::new(0.0, 0.005, 0.0));

    // Flipped version to work around backface culling.
    let mut bottom_arrow = arrows.add_mesh(arrow_mesh, Vector3::new(1.0, 1.0, 1.0));
    bottom_arrow.set_color(1.0, 1.0, 1.0);
    bottom_arrow.set_local_translation(Translation3::new(0.0, 0.005, 0.0));
    bottom_arrow.set_local_rotation(flip_quaternion);
    arrows
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

fn kiss3d_point<C, T>(vector: C) -> Point3<T>
where
    C: Into<Kiss3DCoordinates<T>>,
    T: Scalar,
{
    let vector = vector.into();
    Point3::new(vector.x(), vector.y(), vector.z())
}

// Function to update the arrow's orientation based on new basis vectors
fn update_arrow_orientation(
    arrow: &mut SceneNode,
    x_basis: Point3<f32>,
    y_basis: Point3<f32>,
    z_basis: Point3<f32>,
) {
    let x_basis = Vector3::new(x_basis[0], x_basis[1], x_basis[2]);
    let y_basis = Vector3::new(y_basis[0], y_basis[1], y_basis[2]);
    let z_basis = Vector3::new(z_basis[0], z_basis[1], z_basis[2]);

    // Create a rotation matrix from the orthonormal basis vectors
    let rotation_matrix = Matrix3::from_columns(&[x_basis, y_basis, z_basis]);

    // Convert the rotation matrix to a UnitQuaternion
    let rotation = UnitQuaternion::from_matrix(&rotation_matrix);
    let flip = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_2);

    let rotation = rotation * flip;

    // Apply the rotation to the arrow
    arrow.set_local_rotation(rotation);
}
