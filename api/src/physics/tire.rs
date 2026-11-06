//! Tyre forces: slip angle and slip ratio, both saturating at the friction
//! circle the surface and the wheel load allow.

use super::tuning::{TIRE_LATERAL, TIRE_LONGITUDINAL};

/// Lateral force of one wheel, N. `alpha` is the slip angle in radians (the
/// angle between the wheel and the way it is moving); the force pushes back
/// against that slide.
pub fn lateral(alpha: f64, mu: f64, load: f64) -> f64 {
    let peak = (mu * load).max(0.0);
    -peak * (TIRE_LATERAL * alpha).tanh()
}

/// Longitudinal force of one wheel, N. `slip` is the slip ratio: positive
/// while the wheel spins faster than the car travels.
pub fn longitudinal(slip: f64, mu: f64, load: f64) -> f64 {
    let peak = (mu * load).max(0.0);
    peak * (TIRE_LONGITUDINAL * slip).tanh()
}

/// The slip angle for a lateral and a longitudinal wheel velocity.
pub fn slip_angle(lateral_velocity: f64, longitudinal_velocity: f64) -> f64 {
    lateral_velocity.atan2(longitudinal_velocity.abs().max(0.6))
}

/// The slip ratio for a wheel turning at `omega` (rad/s) with contact-patch
/// speed `speed` (m/s), and radius `radius`.
pub fn slip_ratio(omega: f64, speed: f64, radius: f64) -> f64 {
    (omega * radius - speed) / speed.abs().max(1.0)
}

/// How fast the longitudinal force grows with the slip ratio, at `slip`:
/// the derivative of the saturating curve, 0 at full slide.
pub fn slip_stiffness(slip: f64) -> f64 {
    let t = (TIRE_LONGITUDINAL * slip).tanh();
    TIRE_LONGITUDINAL * (1.0 - t * t)
}

/// Pull a pair of forces back onto the friction circle.
pub fn clamp_circle(longitudinal: &mut f64, lateral: &mut f64, mu: f64, load: f64) {
    let peak = (mu * load).max(0.0);
    let magnitude = (longitudinal.powi(2) + lateral.powi(2)).sqrt();
    if magnitude > peak && magnitude > 1e-9 {
        let scale = peak / magnitude;
        *longitudinal *= scale;
        *lateral *= scale;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::tuning::TIRE_LATERAL;

    #[test]
    fn a_straight_wheel_makes_no_side_force() {
        assert_eq!(lateral(0.0, 1.0, 6000.0), 0.0);
    }

    #[test]
    fn sliding_left_pushes_right() {
        // The car slides to the left (+lateral), so the tyre pushes right.
        assert!(lateral(10.0_f64.atan2(20.0), 1.0, 6000.0) < 0.0);
    }

    #[test]
    fn spinning_faster_than_the_road_pushes_forward() {
        assert!(longitudinal(0.4, 1.0, 3000.0) > 0.0);
        assert!(longitudinal(-0.4, 1.0, 3000.0) < 0.0);
    }

    #[test]
    fn a_side_force_never_exceeds_the_friction_circle() {
        let f = lateral(10.0, 0.5, 3000.0);
        assert!(f.abs() <= 0.5 * 3000.0 + 1e-9);
    }

    #[test]
    fn combining_forces_stays_inside_the_circle() {
        let (mut l, mut s) = (0.5 * 3000.0, 0.5 * 3000.0);
        clamp_circle(&mut l, &mut s, 0.5, 3000.0);
        let magnitude = (l * l + s * s).sqrt();
        assert!((magnitude - 0.5 * 3000.0).abs() < 1e-6);
    }

    #[test]
    fn the_lateral_curve_is_steep_then_flat() {
        let small = lateral(0.01, 1.0, 6000.0);
        let big = lateral(0.5, 1.0, 6000.0);
        assert!(small.abs() < big.abs());
        // Near the peak it barely changes any more.
        let peak = lateral(5.0, 1.0, 6000.0);
        assert!((big - peak).abs() < 0.05 * 6000.0);
        assert!(TIRE_LATERAL > 1.0);
    }

    #[test]
    fn a_locked_wheel_has_a_negative_slip_ratio() {
        let slip = slip_ratio(0.0, 12.0, 0.3445);
        assert!((slip + 12.0 / 12.0).abs() < 1e-9);
    }
}
