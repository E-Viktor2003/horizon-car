//! Vehicle parameters in one place, in the spirit of GTA's handling data.
//! Distances are blocks (metres), angles radians and forces newtons; the tick
//! is 20 Hz with sixteen substeps.

/// Kerb weight, kg.
pub const MASS: f64 = 1200.0;
/// Distance from the centre of mass to the front axle, m.
pub const A: f64 = 1.304;
/// Distance from the centre of mass to the rear axle, m.
pub const B: f64 = 1.338;
pub const WHEELBASE: f64 = A + B;
/// Track width between the wheel centres, m.
pub const TRACK: f64 = 1.522;
/// Height of the centre of mass above the contact plane, m.
pub const CG_HEIGHT: f64 = 0.55;
pub const WHEEL_RADIUS: f64 = 0.3445;
pub const GRAVITY: f64 = 9.81;

/// Side and corner distances used for the inertia of a box.
const WIDTH: f64 = 1.85;
const HEIGHT: f64 = 1.3;
/// Principal moments: roll (about forward), yaw (about up), pitch (about left).
pub const INERTIA: [f64; 3] = [
    MASS * (HEIGHT * HEIGHT + WIDTH * WIDTH) / 12.0,
    MASS * (WHEELBASE * WHEELBASE + WIDTH * WIDTH) / 12.0,
    MASS * (WHEELBASE * WHEELBASE + HEIGHT * HEIGHT) / 12.0,
];

/// Design length of one suspension unit, hub to wheel centre, m.
pub const REST_LENGTH: f64 = 0.20;
/// How far the spring can compress past its design length, m.
pub const MAX_COMPRESSION: f64 = 0.18;
/// How far a wheel may hang below its design length, m.
pub const MAX_DROOP: f64 = 0.12;
/// How much suspension travel is left before the bump stop, m.
pub const BUMP_STOP_AT: f64 = 0.15;
/// Spring rate, N/m. Static compression about 8 cm at the kerb weight.
pub const SPRING_RATE: f64 = 30_000.0;
/// Damper rate, N per (m/s). About 0.5 damping ratio at the corner mass.
pub const DAMPER_RATE: f64 = 2_900.0;
/// Extra spring rate once the bump stop is hit, N/m of travel.
pub const BUMP_STOP_RATE: f64 = 600_000.0;
/// Anti-roll bar rate at one wheel, N/m of left-right compression difference.
pub const ANTI_ROLL_RATE: f64 = 4_500.0;

/// How quickly the tyre builds lateral force, per radian of slip.
pub const TIRE_LATERAL: f64 = 9.0;
/// How quickly the tyre builds longitudinal force, per unit of slip ratio.
pub const TIRE_LONGITUDINAL: f64 = 7.0;
/// Rotational inertia of one wheel and tyre, kg m^2.
pub const WHEEL_INERTIA: f64 = 1.2;

/// Brake torque at one front wheel, N m.
pub const BRAKE_FRONT: f64 = 1_250.0;
/// Brake torque at one rear wheel, N m.
pub const BRAKE_REAR: f64 = 800.0;
/// Handbrake torque at one rear wheel, N m.
pub const HANDBRAKE: f64 = 4_000.0;

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
/// Engine speed a slipping clutch holds while pulling away.
pub const LAUNCH_RPM: f64 = 2_600.0;

/// 0.5 * air density * CdA, N per (m/s)^2.
pub const DRAG: f64 = 0.43;
/// Downforce, N per (m/s)^2.
pub const DOWNFORCE: f64 = 0.6;
/// Rolling resistance as a share of the weight.
pub const ROLLING_RESISTANCE: f64 = 0.015;
/// Electronic limiter, m/s (140 km/h).
pub const MAX_SPEED: f64 = 38.9;

/// Steering lock at a standstill, radians.
pub const MAX_STEER: f64 = 0.55;
/// Speed at which the lock falls to half, m/s.
pub const STEER_FALLOFF: f64 = 30.0;
/// How fast the wheels turn towards the target, rad/s.
pub const STEER_RATE: f64 = 4.0;

/// Surface friction coefficients, keyed by the block under the wheel.
pub fn friction_of(block: &str) -> f64 {
    let name = block.rsplit(':').next().unwrap_or(block);
    if name.contains("ice") || name == "packed_ice" || name == "blue_ice" {
        0.17
    } else if name.contains("snow") {
        0.45
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
