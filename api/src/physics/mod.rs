//! The car's physics: a bicycle model with four wheels, load transfer, a
//! Pacejka-style tyre, a brake split and an automatic gearbox.
//!
//! Everything here is plain `f64` arithmetic with no game types, so the tests
//! run without Horizon.

pub mod engine;
pub mod ground;
pub mod tire;
pub mod tuning;

use engine::Gearbox;
use tuning::*;

/// The driver's pedals and wheel this tick. `steer` is -1 (left) to 1 (right).
#[derive(Clone, Copy, Debug, Default)]
pub struct Controls {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
    pub handbrake: bool,
}

impl Controls {
    pub fn clamped(self) -> Self {
        Self {
            throttle: self.throttle.clamp(0.0, 1.0),
            brake: self.brake.clamp(0.0, 1.0),
            steer: self.steer.clamp(-1.0, 1.0),
            handbrake: self.handbrake,
        }
    }
}

/// What the four wheels stand on this tick, sampled by the server.
#[derive(Clone, Debug)]
pub struct Frame {
    /// Ground height under FL, FR, RL, RR; `f64::NAN` when there is none.
    pub heights: [f64; 4],
    /// Surface friction under each wheel.
    pub friction: [f64; 4],
    /// Where the body should sit (average ground height).
    pub target_y: f64,
    /// The body is touching a wall this tick.
    pub blocked: bool,
}

impl Frame {
    #[allow(dead_code)]
    pub fn flat(y: f64) -> Self {
        Self {
            heights: [y; 4],
            friction: [1.0; 4],
            target_y: y,
            blocked: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    /// The body's feet, in world blocks.
    pub pos: [f64; 3],
    /// Degrees-free yaw in radians: 0 faces +z, positive turns right.
    pub yaw: f64,
    pub vel: [f64; 3],
    pub yaw_rate: f64,
    /// Front wheel angle, radians, positive right.
    pub steer: f64,
    pub spin_f: f64,
    pub spin_r: f64,
    pub pitch: f64,
    pub roll: f64,
    pub ground_pitch: f64,
    pub ground_roll: f64,
    pub airborne: bool,
    pub speed: f64,
    pub gearbox: Gearbox,
}

impl Vehicle {
    pub fn new(pos: [f64; 3], yaw: f64) -> Self {
        Self {
            pos,
            yaw: normalize(yaw),
            vel: [0.0; 3],
            yaw_rate: 0.0,
            steer: 0.0,
            spin_f: 0.0,
            spin_r: 0.0,
            pitch: 0.0,
            roll: 0.0,
            ground_pitch: 0.0,
            ground_roll: 0.0,
            airborne: false,
            speed: 0.0,
            gearbox: Gearbox::default(),
        }
    }

    /// Unit forward vector `(x, z)`.
    pub fn forward(&self) -> [f64; 2] {
        [-self.yaw.sin(), self.yaw.cos()]
    }

    /// Unit right vector `(x, z)`.
    pub fn right(&self) -> [f64; 2] {
        [-self.yaw.cos(), -self.yaw.sin()]
    }

    /// How fast the car is actually going, m/s.
    pub fn forward_speed(&self) -> f64 {
        let f = self.forward();
        self.vel[0] * f[0] + self.vel[2] * f[1]
    }

    /// One physics substep.
    pub fn step(&mut self, controls: &Controls, frame: &Frame, dt: f64) {
        let c = controls.clamped();
        if frame.blocked {
            self.vel[0] = 0.0;
            self.vel[2] = 0.0;
            self.yaw_rate *= 0.3;
        }

        let f = self.forward();
        let r = self.right();
        let v_long = self.vel[0] * f[0] + self.vel[2] * f[1];
        let v_lat = self.vel[0] * r[0] + self.vel[2] * r[1];

        let drive = self.gearbox.update(c.throttle, c.brake, v_long, dt);

        // Steering lock shrinks with speed, as a real rack does.
        let lock = MAX_STEER / (1.0 + (v_long.abs() / STEER_FALLOFF).powi(2));
        let target_steer = c.steer * lock;
        let rate = STEER_RATE * dt;
        self.steer += (target_steer - self.steer).clamp(-rate, rate);

        // Longitudinal forces.
        let mut f_long = drive.force;
        let brake = drive.brake.clamp(0.0, 1.0);
        let brake_force = brake * BRAKE_ACCEL * MASS;
        if v_long.abs() > 0.02 {
            f_long -= v_long.signum() * brake_force;
        }

        // Handbrake: the rear wheels lock and slide.
        let hand = c.handbrake;
        let rear_brake = if hand {
            HANDBRAKE_ACCEL * MASS * 0.5
        } else {
            0.0
        };
        if v_long.abs() > 0.02 {
            f_long -= v_long.signum() * rear_brake;
        }

        // Rolling resistance, aero and engine braking.
        if v_long.abs() > 0.02 {
            f_long -= v_long.signum() * ROLLING_RESISTANCE * MASS * GRAVITY;
            f_long -= DRAG * v_long * v_long.abs();
            if c.throttle < 0.05 && c.brake < 0.05 {
                f_long -= v_long.signum() * 220.0;
            }
        }

        // The electronic limiter simply stops pulling above its speed.
        if v_long > MAX_SPEED {
            f_long = f_long.min(0.0) - 350.0;
        }

        // Axle loads with longitudinal transfer.
        let a_est = f_long / MASS;
        let fz_f =
            (MASS * GRAVITY * B / WHEELBASE - MASS * a_est * CG_HEIGHT / WHEELBASE).max(300.0);
        let fz_r =
            (MASS * GRAVITY * A / WHEELBASE + MASS * a_est * CG_HEIGHT / WHEELBASE).max(300.0);

        let grounded = frame.heights.iter().any(|h| h.is_finite());
        let mut mu_f = if grounded {
            (frame.friction[0] + frame.friction[1]) * 0.5
        } else {
            1.0
        };
        let mut mu_r = if grounded {
            (frame.friction[2] + frame.friction[3]) * 0.5
        } else {
            1.0
        };
        if hand {
            mu_r *= 0.55;
        }
        if !grounded {
            mu_f = 0.0;
            mu_r = 0.0;
        }

        // Lateral tyre forces.
        let vf = v_lat + self.yaw_rate * A;
        let vr = v_lat - self.yaw_rate * B;
        let mut fyf = tire::front(vf, v_long, self.steer, mu_f, fz_f);
        let mut fyr = if hand {
            tire::locked(vr, mu_r, fz_r)
        } else {
            tire::rear(vr, v_long, mu_r, fz_r)
        };
        // Rear wheel lock also drags the car back.
        if hand && v_long.abs() > 0.02 {
            f_long -= v_long.signum() * mu_r * fz_r * 0.6;
        }
        // At a crawl the slip angles are noise.
        if v_long.abs() < 0.5 && c.throttle < 0.05 && c.brake < 0.05 {
            fyf *= 0.25;
            fyr *= 0.25;
            self.yaw_rate *= 0.9;
        }

        let mz = A * fyf - B * fyr;
        self.yaw_rate += mz / INERTIA * dt;
        if frame.blocked {
            self.yaw_rate *= 0.5;
        }

        let a_long = f_long / MASS + GRAVITY * self.ground_pitch.sin();
        let a_lat = (fyf + fyr) / MASS;
        let ax = f[0] * a_long + r[0] * a_lat;
        let az = f[1] * a_long + r[1] * a_lat;
        self.vel[0] += ax * dt;
        self.vel[2] += az * dt;

        // Braking stops the car instead of pushing it backwards.
        if (brake > 0.02 || hand) && v_long.abs() > 0.02 {
            let new_long = self.vel[0] * f[0] + self.vel[2] * f[1];
            if new_long.signum() != v_long.signum() {
                self.vel[0] -= f[0] * new_long;
                self.vel[2] -= f[1] * new_long;
            }
        }

        self.yaw = normalize(self.yaw + self.yaw_rate * dt);
        self.pos[0] += self.vel[0] * dt;
        self.pos[2] += self.vel[2] * dt;

        // Height and attitude follow the ground.
        if grounded {
            self.airborne = false;
            self.vel[1] = 0.0;
            let k = (dt * 12.0).min(1.0);
            self.pos[1] += (frame.target_y - self.pos[1]) * k;
        } else {
            self.airborne = true;
            self.vel[1] -= GRAVITY * dt;
            self.pos[1] += self.vel[1] * dt;
        }

        let h = frame.heights;
        let front = mean(h[0], h[1]);
        let rear = mean(h[2], h[3]);
        let left = mean(h[0], h[2]);
        let right = mean(h[1], h[3]);
        if let (Some(front), Some(rear)) = (front, rear) {
            self.ground_pitch = ((rear - front) / WHEELBASE).atan();
        }
        if let (Some(left), Some(right)) = (left, right) {
            self.ground_roll = ((left - right) / TRACK).atan();
        }

        let target_pitch = self.ground_pitch - a_long * 0.006;
        let target_roll = self.ground_roll - a_lat * 0.006;
        // A step caught by one wheel is not a slope: cap the angle the ground
        // can claim, and let the body settle towards it at a suspension's pace
        // instead of snapping to every block edge it samples.
        const MAX_TILT: f64 = 0.42;
        let target_pitch = target_pitch.clamp(-MAX_TILT, MAX_TILT);
        let target_roll = target_roll.clamp(-MAX_TILT, MAX_TILT);
        let k = (dt * 8.0).min(1.0);
        let max_step = 3.0 * dt;
        self.pitch += ((target_pitch - self.pitch) * k).clamp(-max_step, max_step);
        self.roll += ((target_roll - self.roll) * k).clamp(-max_step, max_step);

        // Wheels roll with the car; the handbrake locks the rear axle.
        self.spin_f = (self.spin_f + v_long / WHEEL_RADIUS * dt).rem_euclid(std::f64::consts::TAU);
        if !hand || v_long.abs() < 0.05 {
            self.spin_r =
                (self.spin_r + v_long / WHEEL_RADIUS * dt).rem_euclid(std::f64::consts::TAU);
        }

        self.speed = (self.vel[0] * self.vel[0] + self.vel[2] * self.vel[2]).sqrt();
        if !self.speed.is_finite() || !self.pos[0].is_finite() || !self.pos[2].is_finite() {
            self.vel = [0.0; 3];
            self.yaw_rate = 0.0;
            self.speed = 0.0;
        }
    }
}

fn mean(a: f64, b: f64) -> Option<f64> {
    match (a.is_finite(), b.is_finite()) {
        (true, true) => Some((a + b) * 0.5),
        (true, false) => Some(a),
        (false, true) => Some(b),
        (false, false) => None,
    }
}

/// Wrap an angle to (-pi, pi].
pub fn normalize(angle: f64) -> f64 {
    let mut a = angle % std::f64::consts::TAU;
    if a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step(car: &mut Vehicle, c: Controls, seconds: f64) {
        let frame = Frame::flat(0.0);
        let dt = 0.0125;
        let mut t = 0.0;
        while t < seconds {
            car.step(&c, &frame, dt);
            t += dt;
        }
    }

    #[test]
    fn full_throttle_accelerates_and_shifts_up() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        step(
            &mut car,
            Controls {
                throttle: 1.0,
                ..Default::default()
            },
            4.0,
        );
        assert!(car.forward_speed() > 8.0, "speed {}", car.forward_speed());
        assert!(car.gearbox.gear >= 2);
        assert!(car.pos[2] > 5.0);
    }

    #[test]
    fn braking_from_speed_stops_the_car() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        car.vel[2] = 20.0;
        step(
            &mut car,
            Controls {
                brake: 1.0,
                ..Default::default()
            },
            4.0,
        );
        assert!(
            car.forward_speed().abs() < 0.2,
            "speed {}",
            car.forward_speed()
        );
    }

    #[test]
    fn steering_right_turns_right() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        car.vel[2] = 10.0;
        step(
            &mut car,
            Controls {
                throttle: 0.3,
                steer: 1.0,
                ..Default::default()
            },
            1.0,
        );
        assert!(car.yaw > 0.0, "yaw {}", car.yaw);
        assert!(car.pos[0] < 0.0, "x {}", car.pos[0]);
    }

    #[test]
    fn the_handbrake_slides_the_rear() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        car.vel[2] = 15.0;
        let before = car.yaw_rate;
        step(
            &mut car,
            Controls {
                handbrake: true,
                steer: 1.0,
                ..Default::default()
            },
            0.5,
        );
        assert!(car.yaw_rate > before);
        assert!(car.forward_speed() < 15.0);
    }

    #[test]
    fn an_uphill_slope_slows_the_car() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        car.vel[2] = 10.0;
        let mut frame = Frame::flat(0.0);
        frame.heights = [1.3, 1.3, 0.0, 0.0];
        frame.target_y = 0.65;
        for _ in 0..80 {
            car.step(&Controls::default(), &frame, 0.0125);
        }
        assert!(car.forward_speed() < 10.0);
    }

    #[test]
    fn ten_thousand_random_steps_stay_finite() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        let mut seed = 0x1234_5678_9abc_def0_u64;
        for i in 0..10_000 {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = |shift: u32| ((seed >> shift) & 0xffff) as f64 / 65535.0;
            let c = Controls {
                throttle: r(0),
                brake: r(16) * 0.5,
                steer: r(32) * 2.0 - 1.0,
                handbrake: r(48) > 0.8,
            };
            let mut frame = Frame::flat(64.0);
            frame.friction = [r(4) * 1.0, r(8) * 1.0, r(12) * 1.0, r(20) * 1.0];
            car.step(&c, &frame, 0.0125);
            assert!(
                car.pos.iter().all(|v| v.is_finite())
                    && car.vel.iter().all(|v| v.is_finite())
                    && car.yaw.is_finite()
                    && car.yaw_rate.is_finite()
                    && car.pitch.is_finite()
                    && car.roll.is_finite(),
                "step {i} went non-finite"
            );
        }
    }

    #[test]
    fn air_drag_bounds_the_top_speed() {
        let mut car = Vehicle::new([0.0; 3], 0.0);
        step(
            &mut car,
            Controls {
                throttle: 1.0,
                ..Default::default()
            },
            90.0,
        );
        assert!(car.forward_speed() < MAX_SPEED + 2.0);
        assert!(car.forward_speed() > 20.0);
    }
}
