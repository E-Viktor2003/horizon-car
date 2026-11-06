//! The car's chassis as a rigid body: position, velocity, orientation and
//! angular velocity, with forces accumulated and integrated over a substep.
//!
//! Orientation is kept as the three body axes in world space (`left`, `up`,
//! `forward`, matching the model's +x left, +y up, +z forward) instead of a
//! quaternion: the axes are re-orthonormalized every step, which stays stable
//! for a car and makes the render's yaw/pitch/roll trivial to read back.

use super::vec::V3;

#[derive(Clone, Debug)]
pub struct Body {
    /// Centre of mass, world blocks.
    pub pos: V3,
    pub vel: V3,
    /// Body axes in world space.
    pub left: V3,
    pub up: V3,
    pub forward: V3,
    /// World angular velocity, rad/s.
    pub omega: V3,
    pub mass: f64,
    /// Principal moments of inertia in body axes: roll (forward), yaw (up),
    /// pitch (left).
    pub inertia: V3,
    /// Forces and torques collected for the next substep.
    pub force: V3,
    pub torque: V3,
}

impl Body {
    pub fn new(pos: V3, yaw: f64, mass: f64, inertia: V3) -> Self {
        let (left, up, forward) = axes(yaw);
        Self {
            pos,
            vel: V3::ZERO,
            left,
            up,
            forward,
            omega: V3::ZERO,
            mass,
            inertia,
            force: V3::ZERO,
            torque: V3::ZERO,
        }
    }

    /// A body-space vector (x left, y up, z forward) in world space.
    pub fn to_world(&self, local: V3) -> V3 {
        self.left * local.x + self.up * local.y + self.forward * local.z
    }

    /// A world-space vector in body coordinates.
    pub fn to_body(&self, world: V3) -> V3 {
        V3::new(
            world.dot(self.left),
            world.dot(self.up),
            world.dot(self.forward),
        )
    }

    /// World position of a body-space point.
    pub fn point(&self, local: V3) -> V3 {
        self.pos + self.to_world(local)
    }

    /// Add `force` acting at world `point`.
    pub fn add_force_at(&mut self, force: V3, point: V3) {
        self.force += force;
        self.torque += (point - self.pos).cross(force);
    }

    /// Add a force through the centre of mass.
    pub fn add_force(&mut self, force: V3) {
        self.force += force;
    }

    /// Velocity of the body at world `point`.
    pub fn velocity_at(&self, point: V3) -> V3 {
        self.vel + self.omega.cross(point - self.pos)
    }

    /// Semi-implicit Euler: velocities first, then position and orientation.
    pub fn integrate(&mut self, dt: f64, gravity: f64) {
        let accel = self.force / self.mass + V3::new(0.0, -gravity, 0.0);
        self.vel += accel * dt;
        self.pos += self.vel * dt;

        // Euler's equations in the body frame, with the principal axes.
        let w = self.to_body(self.omega);
        let iw = V3::new(
            self.inertia.x * w.x,
            self.inertia.y * w.y,
            self.inertia.z * w.z,
        );
        let tau = self.to_body(self.torque);
        let gyro = w.cross(iw);
        let dw = V3::new(
            (tau.x - gyro.x) / self.inertia.x,
            (tau.y - gyro.y) / self.inertia.y,
            (tau.z - gyro.z) / self.inertia.z,
        );
        let w = w + dw * dt;
        self.omega = self.to_world(w);

        self.turn(self.omega * dt);
        self.force = V3::ZERO;
        self.torque = V3::ZERO;
    }

    /// Rotate the body by the world-space rotation vector `r` (axis times
    /// angle), then clean the axes up.
    fn turn(&mut self, r: V3) {
        let angle = r.length();
        if angle > 1e-12 {
            let axis = r / angle;
            let (s, c) = angle.sin_cos();
            for v in [&mut self.left, &mut self.up, &mut self.forward] {
                *v = *v * c + axis.cross(*v) * s + axis * (axis.dot(*v) * (1.0 - c));
            }
        }
        self.forward = self.forward.normalized();
        self.left = self.left.without(self.forward).normalized();
        self.up = self.forward.cross(self.left).normalized();
        if !self.forward.is_finite() || !self.left.is_finite() || !self.up.is_finite() {
            // A blow-up would poison everything after it: reset upright.
            let (left, up, forward) = axes(self.yaw());
            self.left = left;
            self.up = up;
            self.forward = forward;
            self.omega = V3::ZERO;
        }
    }

    /// Yaw in radians, 0 facing +z, positive turning right.
    pub fn yaw(&self) -> f64 {
        (-self.forward.x).atan2(self.forward.z)
    }

    /// Pitch in radians, positive with the nose down, as the render draws it.
    pub fn pitch(&self) -> f64 {
        let level = (self.forward.x * self.forward.x + self.forward.z * self.forward.z).sqrt();
        (-self.forward.y).atan2(level)
    }

    /// Roll in radians, positive lifting the left side, as the render draws it.
    pub fn roll(&self) -> f64 {
        self.left.y.atan2(self.up.y)
    }

    #[allow(dead_code)]
    pub fn set_yaw(&mut self, yaw: f64) {
        let (left, up, forward) = axes(yaw);
        self.left = left;
        self.up = up;
        self.forward = forward;
    }
}

/// Unit body axes for a yaw: +x left, +y up, +z forward.
pub fn axes(yaw: f64) -> (V3, V3, V3) {
    let (s, c) = yaw.sin_cos();
    let forward = V3::new(-s, 0.0, c);
    let left = V3::new(c, 0.0, s);
    (left, V3::new(0.0, 1.0, 0.0), forward)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body() -> Body {
        let inertia = V3::new(1700.0, 1900.0, 500.0);
        Body::new(V3::new(0.0, 1.0, 0.0), 0.0, 1200.0, inertia)
    }

    #[test]
    fn the_axes_match_the_model_convention() {
        let b = body();
        assert_eq!(b.forward, V3::new(0.0, 0.0, 1.0));
        assert_eq!(b.left, V3::new(1.0, 0.0, 0.0));
        assert_eq!(b.up, V3::new(0.0, 1.0, 0.0));
        assert_eq!(b.forward.cross(b.left), b.up);
    }

    #[test]
    fn yaw_round_trips_through_the_axes() {
        for deg in [-170.0f64, -90.0, -10.0, 0.0, 35.0, 90.0, 179.0] {
            let mut b = body();
            let yaw = deg.to_radians();
            b.set_yaw(yaw);
            assert!((b.yaw() - yaw).abs() < 1e-9, "{deg}: {}", b.yaw());
            assert!(b.pitch().abs() < 1e-9);
            assert!(b.roll().abs() < 1e-9);
        }
    }

    #[test]
    fn gravity_drops_a_free_body() {
        let mut b = body();
        for _ in 0..10 {
            b.integrate(0.01, 9.81);
        }
        assert!(b.vel.y < -0.9);
        assert!(b.pos.y < 0.95);
    }

    #[test]
    fn a_force_off_centre_spins_the_body() {
        let mut b = body();
        // Push the nose to the right: a yaw to the left.
        b.add_force_at(V3::new(-1000.0, 0.0, 0.0), V3::new(0.0, 1.0, 2.0));
        b.integrate(0.01, 0.0);
        assert!(b.omega.y.abs() > 1e-6);
    }

    #[test]
    fn nose_up_is_a_negative_pitch() {
        let mut b = body();
        // Turn the nose up by 10 degrees about the left axis.
        b.turn(V3::new(-0.1745, 0.0, 0.0));
        assert!((b.pitch() + 0.1745).abs() < 1e-3, "{}", b.pitch());
    }

    #[test]
    fn the_body_keeps_its_axes_orthonormal_as_it_turns() {
        let mut b = body();
        for _ in 0..500 {
            b.turn(V3::new(0.01, 0.02, -0.013));
        }
        assert!((b.forward.length() - 1.0).abs() < 1e-9);
        assert!((b.left.length() - 1.0).abs() < 1e-9);
        assert!((b.up.length() - 1.0).abs() < 1e-9);
        assert!(b.forward.dot(b.left).abs() < 1e-9);
        assert!(b.forward.dot(b.up).abs() < 1e-9);
        assert!(b.left.dot(b.up).abs() < 1e-9);
    }
}
