//! Tyre forces: a slip-angle curve with a friction ellipse.

use super::tuning::{STIFFNESS_FRONT, STIFFNESS_REAR};

/// Lateral force of one axle for a slip angle, clamped to `mu * load`.
///
/// `slip` is the angle between the wheel and its velocity, positive for a
/// wheel that slides to the left of where it points. `mu` is the surface
/// grip and `load` the axle's normal force.
pub fn lateral(slip: f64, stiffness: f64, mu: f64, load: f64) -> f64 {
    let peak = (mu * load).max(0.0);
    (-stiffness * slip).clamp(-peak, peak)
}

/// Front-axle slip angle and force.
pub fn front(slip_velocity: f64, long_velocity: f64, steer: f64, mu: f64, load: f64) -> f64 {
    let speed = long_velocity.abs().max(0.7);
    let angle = (slip_velocity / speed).atan() - steer;
    lateral(angle, STIFFNESS_FRONT, mu, load)
}

/// Rear-axle slip angle and force.
pub fn rear(slip_velocity: f64, long_velocity: f64, mu: f64, load: f64) -> f64 {
    let speed = long_velocity.abs().max(0.7);
    let angle = (slip_velocity / speed).atan();
    lateral(angle, STIFFNESS_REAR, mu, load)
}

/// A locked wheel slides: the force is kinetic friction against its motion.
pub fn locked(velocity: f64, mu: f64, load: f64) -> f64 {
    let peak = (mu * load).max(0.0);
    if velocity.abs() < 0.05 {
        0.0
    } else {
        -velocity.signum() * peak
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_straight_wheel_makes_no_force() {
        assert_eq!(front(0.0, 20.0, 0.0, 1.0, 6000.0), 0.0);
    }

    #[test]
    fn steering_right_pushes_the_front_right() {
        // A right-pointing wheel (positive steer) with no slip velocity.
        assert!(front(0.0, 20.0, 0.1, 1.0, 6000.0) > 0.0);
    }

    #[test]
    fn sliding_left_pushes_right() {
        assert!(lateral(0.2, STIFFNESS_FRONT, 1.0, 6000.0) < 0.0);
    }

    #[test]
    fn the_force_never_exceeds_the_friction_circle() {
        let f = lateral(10.0, STIFFNESS_FRONT, 0.5, 3000.0);
        assert!(f >= -0.5 * 3000.0 && f <= 0.5 * 3000.0);
    }
}
