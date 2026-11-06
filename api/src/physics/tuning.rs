//! Vehicle parameters in one place. Distances are blocks (metres), angles
//! radians and forces newtons; the tick is 20 Hz with four substeps.

/// Kerb weight, kg.
pub const MASS: f64 = 1200.0;
/// Distance from the centre of mass to the front axle, m.
pub const A: f64 = 1.304;
/// Distance from the centre of mass to the rear axle, m.
pub const B: f64 = 1.338;
pub const WHEELBASE: f64 = A + B;
/// Track width between the wheel centres, m.
pub const TRACK: f64 = 1.522;
pub const CG_HEIGHT: f64 = 0.5;
pub const WHEEL_RADIUS: f64 = 0.3445;
/// Yaw inertia of a box, kg m^2.
pub const INERTIA: f64 = MASS * (WHEELBASE * WHEELBASE + 1.85 * 1.85) / 12.0;
pub const GRAVITY: f64 = 9.81;

/// Steering lock at a standstill, radians.
pub const MAX_STEER: f64 = 0.55;
/// Speed at which the lock falls to half, m/s.
pub const STEER_FALLOFF: f64 = 18.0;
/// How fast the wheels turn towards the target, rad/s.
pub const STEER_RATE: f64 = 4.0;

pub const IDLE_RPM: f64 = 850.0;
pub const REDLINE_RPM: f64 = 6500.0;
pub const PEAK_TORQUE: f64 = 240.0;
pub const FINAL_DRIVE: f64 = 3.7;
pub const DRIVETRAIN_EFFICIENCY: f64 = 0.9;
pub const GEAR_RATIOS: [f64; 7] = [0.0, 3.6, 2.2, 1.5, 1.1, 0.85, 0.68];
pub const REVERSE_RATIO: f64 = 3.4;
/// Automatic shift points, rpm.
pub const UPSHIFT_RPM: f64 = 5700.0;
pub const DOWNSHIFT_RPM: f64 = 1450.0;
pub const SHIFT_TIME: f64 = 0.3;

/// Largest braking deceleration, m/s^2 (all four wheels).
pub const BRAKE_ACCEL: f64 = 11.0;
pub const HANDBRAKE_ACCEL: f64 = 7.0;
pub const ROLLING_RESISTANCE: f64 = 0.015;
/// 0.5 * air density * CdA, N per (m/s)^2.
pub const DRAG: f64 = 0.43;
/// Electronic limiter, m/s (140 km/h).
pub const MAX_SPEED: f64 = 38.9;

/// Cornering stiffness, N/rad.
pub const STIFFNESS_FRONT: f64 = 90_000.0;
pub const STIFFNESS_REAR: f64 = 105_000.0;

/// Surface friction coefficients, keyed by the block under the wheel.
pub fn friction_of(block: &str) -> f64 {
    let name = block.rsplit(':').next().unwrap_or(block);
    if name.contains("ice") || name == "packed_ice" || name == "blue_ice" {
        0.17
    } else if name.contains("snow") {
        0.35
    } else if name.contains("sand") || name.contains("gravel") {
        0.65
    } else if name.contains("grass") || name.contains("dirt") || name.contains("mud") {
        0.82
    } else if name.contains("slime") {
        0.9
    } else {
        1.0
    }
}
