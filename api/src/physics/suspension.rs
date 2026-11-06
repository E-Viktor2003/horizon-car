//! The four suspension units: where they attach to the chassis, and what the
//! spring and damper do for a given compression.

use super::tuning::{
    A, ANTI_ROLL_RATE, B, BUMP_STOP_AT, BUMP_STOP_RATE, CG_HEIGHT, DAMPER_RATE, MAX_COMPRESSION,
    MAX_DROOP, REST_LENGTH, SPRING_RATE, TRACK, WHEEL_RADIUS,
};
use super::vec::V3;

/// Height of a hub above the model's feet at the design length.
pub const HUB_HEIGHT: f64 = WHEEL_RADIUS + REST_LENGTH;

/// One wheel's mounting point in body coordinates, relative to the centre of
/// mass. Index order is FL, FR, RL, RR, the models' order.
#[derive(Clone, Copy, Debug)]
pub struct Mount {
    pub front: bool,
    pub local: V3,
}

const HALF_TRACK: f64 = TRACK * 0.5;
const HUB_Y: f64 = HUB_HEIGHT - CG_HEIGHT;

pub const MOUNTS: [Mount; 4] = [
    Mount {
        front: true,
        local: V3::new(HALF_TRACK, HUB_Y, A),
    },
    Mount {
        front: true,
        local: V3::new(-HALF_TRACK, HUB_Y, A),
    },
    Mount {
        front: false,
        local: V3::new(HALF_TRACK, HUB_Y, -B),
    },
    Mount {
        front: false,
        local: V3::new(-HALF_TRACK, HUB_Y, -B),
    },
];

/// How far the hub may be from the ground before a wheel stops reaching it.
pub const MAX_RAY: f64 = REST_LENGTH + MAX_DROOP + WHEEL_RADIUS;
/// Shortest suspension length the bump stop allows.
pub const MIN_LENGTH: f64 = REST_LENGTH - MAX_COMPRESSION;

/// The suspension length for a hub `drop` above the ground, clamped to what
/// the unit allows.
pub fn length_for(drop: f64) -> f64 {
    (drop - WHEEL_RADIUS).clamp(MIN_LENGTH, REST_LENGTH + MAX_DROOP)
}

/// The compression of a unit of `length` (positive pushes the wheel up).
pub fn compression_for(length: f64) -> f64 {
    REST_LENGTH - length
}

/// Spring and damper force along the body's up axis for a compression and its
/// rate (positive when the suspension is being squeezed).
pub fn spring_force(compression: f64, rate: f64) -> f64 {
    if compression <= 0.0 {
        return 0.0;
    }
    let mut force = SPRING_RATE * compression + DAMPER_RATE * rate;
    if compression > BUMP_STOP_AT {
        force += BUMP_STOP_RATE * (compression - BUMP_STOP_AT);
    }
    force.max(0.0)
}

/// Extra up force on the more compressed wheel of an axle and the matching
/// down force on the other, as an anti-roll bar makes.
pub fn anti_roll(left: f64, right: f64) -> f64 {
    ANTI_ROLL_RATE * (left - right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mounts_sit_in_the_model_arch() {
        // Front-left is ahead and to the left (+x) of the centre of mass.
        assert!(MOUNTS[0].front);
        assert!(MOUNTS[0].local.z > 0.0);
        assert!(MOUNTS[0].local.x > 0.0);
        assert!(!MOUNTS[2].front);
        assert!(MOUNTS[2].local.z < 0.0);
        // Left and right are mirror images.
        assert!((MOUNTS[0].local.x + MOUNTS[1].local.x).abs() < 1e-12);
    }

    #[test]
    fn a_full_load_compresses_the_spring() {
        let force = spring_force(0.08, 0.0);
        assert!((force - SPRING_RATE * 0.08).abs() < 1e-6, "{force}");
        assert!(force > 2_000.0 && force < 4_000.0, "{force}");
    }

    #[test]
    fn a_drooping_wheel_makes_no_force() {
        assert_eq!(spring_force(-0.05, 0.0), 0.0);
    }

    #[test]
    fn compressing_faster_pushes_harder() {
        assert!(spring_force(0.08, 0.5) > spring_force(0.08, 0.0));
    }

    #[test]
    fn the_bump_stop_stiffens_the_end_of_the_travel() {
        assert!(spring_force(0.17, 0.0) > SPRING_RATE * 0.17 + 5_000.0);
    }

    #[test]
    fn the_ray_reaches_the_design_length_plus_droop() {
        assert!((MAX_RAY - (REST_LENGTH + MAX_DROOP + WHEEL_RADIUS)).abs() < 1e-12);
        // A hub exactly MAX_RAY above the ground is a drooped wheel.
        let length = length_for(MAX_RAY);
        assert!((length - (REST_LENGTH + MAX_DROOP)).abs() < 1e-12);
    }
}
