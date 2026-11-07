//! The car's physics: a rigid chassis on four raycast suspension units, with
//! slip tyres, wheel spin, an engine, a clutch and an automatic gearbox.
//!
//! Everything here is plain `f64` arithmetic with no game types, so the tests
//! run without Horizon.

pub mod body;
pub mod engine;
pub mod ground;
pub mod suspension;
pub mod tire;
pub mod tuning;
pub mod vec;

use body::Body;
use engine::Gearbox;
use tuning::*;
use vec::V3;

/// Substeps per 20 Hz server tick. The springs are stiff enough that one
/// 50 ms step would explode; 16 substeps is 320 Hz.
pub const SUBSTEPS: usize = 16;
/// Length of one substep, seconds.
pub const STEP: f64 = 0.05 / SUBSTEPS as f64;

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

/// What the four wheels stand on this tick, sampled by the server from the
/// world before the physics steps. The height is the top of the ground under
/// each hub; `None` means nothing within the suspension's reach.
#[derive(Clone, Debug)]
pub struct Ground {
    pub surfaces: [Option<f64>; 4],
    pub friction: [f64; 4],
}

impl Ground {
    #[allow(dead_code)]
    pub fn flat(y: f64) -> Self {
        Self {
            surfaces: [Some(y); 4],
            friction: [1.0; 4],
        }
    }
}

/// One wheel: where it hangs, how fast it turns and what it carries.
#[derive(Clone, Debug)]
pub struct Wheel {
    /// Mounting point in body coordinates, relative to the centre of mass.
    pub mount: V3,
    pub front: bool,
    /// Angle of the wheel around its axle, radians (for drawing).
    pub spin: f64,
    /// Angular velocity, rad/s (positive rolls the car forward).
    pub omega: f64,
    /// Suspension compression, m (positive pushes the wheel up).
    pub compression: f64,
    /// Normal force the ground carries through this wheel, N.
    pub load: f64,
    pub grounded: bool,
    /// Steering angle, radians, positive to the right.
    pub steer: f64,
}

impl Wheel {
    fn new(mount: suspension::Mount) -> Self {
        Self {
            mount: mount.local,
            front: mount.front,
            spin: 0.0,
            omega: 0.0,
            compression: 0.0,
            load: 0.0,
            grounded: false,
            steer: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub body: Body,
    pub wheels: [Wheel; 4],
    /// Front wheel angle at the rack, radians, positive right.
    pub steer: f64,
    pub gearbox: Gearbox,
    pub airborne: bool,
    /// How hard the tyres are sliding, 0 grip to 1 fully sliding.
    pub slip: f64,
}

impl Vehicle {
    pub fn new(feet: [f64; 3], yaw: f64) -> Self {
        let (_, up, _) = body::axes(yaw);
        let pos = V3::from_array(feet) + up * CG_HEIGHT;
        let inertia = V3::new(INERTIA[0], INERTIA[1], INERTIA[2]);
        Self {
            body: Body::new(pos, yaw, MASS, inertia),
            wheels: suspension::MOUNTS.map(Wheel::new),
            steer: 0.0,
            gearbox: Gearbox::default(),
            airborne: true,
            slip: 0.0,
        }
    }

    /// How far the body squats on its springs, averaged over the wheels.
    /// The render shifts the model down by this so the wheels stay on the
    /// ground while the body sits on the suspension.
    pub fn ride(&self) -> f64 {
        self.wheels.iter().map(|w| w.compression).sum::<f64>() / 4.0
    }

    /// The contact plane the wheels stand on: this is where the pig anchor
    /// goes, so the entity never sits inside the ground.
    pub fn origin(&self) -> [f64; 3] {
        (self.body.pos - self.body.up * (CG_HEIGHT - self.ride())).to_array()
    }

    /// Move the chassis so its contact plane sits at `feet`.
    pub fn set_origin(&mut self, feet: [f64; 3]) {
        self.body.pos = V3::from_array(feet) + self.body.up * (CG_HEIGHT - self.ride());
    }

    pub fn yaw(&self) -> f64 {
        self.body.yaw()
    }

    pub fn pitch(&self) -> f64 {
        self.body.pitch()
    }

    pub fn roll(&self) -> f64 {
        self.body.roll()
    }

    /// World positions of the four mounting points.
    pub fn hubs(&self) -> [V3; 4] {
        suspension::MOUNTS.map(|m| self.body.point(m.local))
    }

    pub fn forward_speed(&self) -> f64 {
        self.body.vel.dot(self.body.forward)
    }

    /// The angle of each front wheel after Ackermann correction: the inside
    /// wheel turns tighter than the outside one.
    fn ackermann(steer: f64) -> (f64, f64) {
        if steer.abs() < 1e-5 {
            return (0.0, 0.0);
        }
        let radius = WHEELBASE / steer.tan().abs();
        let inner = (WHEELBASE / (radius - TRACK * 0.5).max(0.2)).atan();
        let outer = (WHEELBASE / (radius + TRACK * 0.5)).atan();
        let sign = steer.signum();
        if steer > 0.0 {
            // Turning right: the right wheel is the inside one.
            (outer * sign, inner * sign)
        } else {
            (inner * sign, outer * sign)
        }
    }

    /// One physics substep.
    pub fn step(&mut self, controls: &Controls, ground: &Ground, dt: f64) {
        let c = controls.clamped();

        // The gearbox reads the car's speed: wheelspin would make it hunt.
        let drive = self
            .gearbox
            .update(c.throttle, c.brake, self.forward_speed(), dt);

        // Steering rack: the lock shrinks with speed, as a real one does.
        let lock = MAX_STEER / (1.0 + (self.forward_speed().abs() / STEER_FALLOFF).powi(2));
        let target = c.steer * lock;
        let rate = STEER_RATE * dt;
        self.steer += (target - self.steer).clamp(-rate, rate);
        let (steer_left, steer_right) = Self::ackermann(self.steer);
        self.wheels[0].steer = steer_left;
        self.wheels[1].steer = steer_right;
        self.wheels[2].steer = 0.0;
        self.wheels[3].steer = 0.0;

        let up = self.body.up;
        let mut compression = [0.0; 4];
        let mut grounded = [false; 4];

        // 1. Suspension: a spring and damper along the body's up axis.
        for i in 0..4 {
            let mount = self.wheels[i].mount;
            let hub = self.body.point(mount);
            let compression_now = match ground.surfaces[i] {
                Some(surface) if hub.y - surface <= suspension::MAX_RAY => {
                    let length = suspension::length_for(hub.y - surface);
                    let squeezed = suspension::compression_for(length);
                    let rate = -self.body.velocity_at(hub).dot(up);
                    let force = suspension::spring_force(squeezed, rate);
                    self.body.add_force_at(up * force, hub);
                    self.wheels[i].load = force;
                    self.wheels[i].grounded = true;
                    squeezed
                }
                _ => {
                    self.wheels[i].load = 0.0;
                    self.wheels[i].grounded = false;
                    -MAX_DROOP
                }
            };
            self.wheels[i].compression = compression_now;
            compression[i] = compression_now;
            grounded[i] = self.wheels[i].grounded;
        }

        // 2. Anti-roll bars: transfer load across each axle.
        for axle in [0, 2] {
            let difference = compression[axle] - compression[axle + 1];
            let force = suspension::anti_roll(compression[axle], compression[axle + 1]);
            if difference.abs() > 1e-6 {
                if grounded[axle] {
                    let hub = self.body.point(self.wheels[axle].mount);
                    self.body.add_force_at(up * force, hub);
                    self.wheels[axle].load += force;
                }
                if grounded[axle + 1] {
                    let hub = self.body.point(self.wheels[axle + 1].mount);
                    self.body.add_force_at(up * -force, hub);
                    self.wheels[axle + 1].load -= force;
                }
            }
        }

        // 3. Tyres and wheel spin.
        let mut sliding = 0.0f64;
        for i in 0..4 {
            let wheel = &self.wheels[i];
            let mount = wheel.mount;
            let front = wheel.front;
            let steer = wheel.steer;
            let omega = wheel.omega;
            let load = wheel.load.max(0.0);
            let Some(surface) = ground.surfaces[i] else {
                continue;
            };
            if !wheel.grounded || load <= 0.0 {
                // A wheel in the air only has its own inertia: the engine can
                // still spin it up.
                let drive_torque = if front {
                    0.0
                } else {
                    drive.force * WHEEL_RADIUS * 0.5
                };
                self.wheels[i].omega += drive_torque / WHEEL_INERTIA * dt;
                self.wheels[i].spin = (self.wheels[i].spin + self.wheels[i].omega * dt)
                    .rem_euclid(std::f64::consts::TAU);
                continue;
            }

            let hub = self.body.point(mount);
            let contact = V3::new(hub.x, surface, hub.z);
            // The wheel plane: forward turned towards the right by `steer`.
            let forward = self.body.forward;
            let left = self.body.left;
            let wheel_forward = forward * steer.cos() - left * steer.sin();
            let wheel_left = up.cross(wheel_forward);
            let velocity = self.body.velocity_at(contact);
            let v_long = velocity.dot(wheel_forward);
            let v_lat = velocity.dot(wheel_left);

            let mu = ground.friction[i];
            let alpha = tire::slip_angle(v_lat, v_long);
            let drive_torque = if front {
                0.0
            } else {
                drive.force * WHEEL_RADIUS * 0.5
            };

            // The tyre's reaction feeds back into the wheel, and at low speed
            // that feedback is far stiffer than an explicit step can take.
            // Solve the wheel's spin implicitly for the linearised tyre, then
            // take the force at the new spin.
            let slip = tire::slip_ratio(omega, v_long, WHEEL_RADIUS);
            let f0 = tire::longitudinal(slip, mu, load);
            let response =
                tire::slip_stiffness(slip) * mu * load * WHEEL_RADIUS / v_long.abs().max(1.0);
            let mut wheel_omega = (WHEEL_INERTIA / dt * omega + drive_torque - f0 * WHEEL_RADIUS
                + response * WHEEL_RADIUS * omega)
                / (WHEEL_INERTIA / dt + response * WHEEL_RADIUS);

            // Brakes: a torque against the spin, never turning it backwards.
            let mut brake = drive.brake * if front { BRAKE_FRONT } else { BRAKE_REAR };
            if !front && c.handbrake {
                brake += HANDBRAKE;
            }
            if brake > 0.0 {
                let delta = brake / WHEEL_INERTIA * dt;
                if wheel_omega.abs() <= delta {
                    wheel_omega = 0.0;
                } else {
                    wheel_omega -= wheel_omega.signum() * delta;
                }
            }
            self.wheels[i].omega = wheel_omega;
            self.wheels[i].spin =
                (self.wheels[i].spin + wheel_omega * dt).rem_euclid(std::f64::consts::TAU);

            let slip = tire::slip_ratio(wheel_omega, v_long, WHEEL_RADIUS);
            let mut f_long = tire::longitudinal(slip, mu, load);
            let mut f_lat = tire::lateral(alpha, mu, load);
            tire::clamp_circle(&mut f_long, &mut f_lat, mu, load);
            let force = wheel_forward * f_long + wheel_left * f_lat;
            self.body.add_force_at(force, contact);

            // How far the tyre is from gripping, for the HUD and the skid sound.
            let used = (f_long * f_long + f_lat * f_lat).sqrt() / (mu * load).max(1.0);
            sliding = sliding.max(used);
        }
        self.slip = (self.slip * 0.85 + sliding.min(1.5) * 0.15).clamp(0.0, 1.0);

        // 4. Aerodynamics, rolling resistance and the limiter.
        let speed = self.body.vel.length();
        if speed > 0.05 {
            self.body.add_force(self.body.vel * (-DRAG * speed));
            self.body.add_force(-up * (DOWNFORCE * speed * speed));
        }
        let grounded_any = grounded.iter().any(|g| *g);
        if grounded_any {
            let forward_speed = self.forward_speed();
            if forward_speed.abs() > 0.05 {
                self.body.add_force(
                    self.body.forward
                        * (-forward_speed.signum() * ROLLING_RESISTANCE * MASS * GRAVITY),
                );
            }
        }
        // At a crawl with no pedals the tyres hold the car. Without this a
        // free-rolling wheel cannot resist the small sideways part of the
        // suspension force and the car creeps away.
        if grounded_any && c.throttle < 0.05 && c.brake < 0.05 {
            let horizontal = V3::new(self.body.vel.x, 0.0, self.body.vel.z);
            if horizontal.length() < 1.0 {
                self.body.add_force(-horizontal * 9_000.0);
            }
        }

        if self.forward_speed() > MAX_SPEED {
            let excess = self.body.vel.dot(self.body.forward) - MAX_SPEED;
            self.body.vel -= self.body.forward * excess;
        }

        self.body.integrate(dt, GRAVITY);

        // Nothing physical may leave the world as a NaN: stop and start over.
        if !self.body.pos.is_finite() || !self.body.vel.is_finite() {
            self.body.vel = V3::ZERO;
            self.body.omega = V3::ZERO;
        }
        self.airborne = !grounded.iter().any(|g| *g);
    }
}

/// Wrap an angle to (-pi, pi].
pub fn normalize(angle: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let wrapped = (angle + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI;
    if wrapped <= -std::f64::consts::PI {
        wrapped + two_pi
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settle(vehicle: &mut Vehicle, ground: &Ground, controls: Controls) {
        for _ in 0..(20 * 4) {
            for _ in 0..SUBSTEPS {
                vehicle.step(&controls, ground, STEP);
            }
        }
    }

    fn driven(vehicle: &mut Vehicle, ground: &Ground, controls: Controls, seconds: f64) -> f64 {
        let steps = (seconds / 0.05) as usize;
        for _ in 0..steps {
            for _ in 0..SUBSTEPS {
                vehicle.step(&controls, ground, STEP);
            }
        }
        vehicle.forward_speed()
    }

    fn car() -> Vehicle {
        Vehicle::new([0.0, 0.0, 0.0], 0.0)
    }

    #[test]
    fn the_car_settles_on_its_springs() {
        let mut v = car();
        settle(&mut v, &Ground::flat(0.0), Controls::default());
        assert!(v.airborne == false);
        for wheel in &v.wheels {
            assert!(wheel.grounded);
            assert!(
                (wheel.compression - 0.081).abs() < 0.02,
                "{}",
                wheel.compression
            );
            assert!(wheel.load > 2000.0 && wheel.load < 4000.0, "{}", wheel.load);
        }
        // And it stays put.
        let origin = v.origin();
        settle(&mut v, &Ground::flat(0.0), Controls::default());
        let moved =
            ((v.origin()[0] - origin[0]).powi(2) + (v.origin()[2] - origin[2]).powi(2)).sqrt();
        assert!(moved < 0.05, "moved {moved}");
        assert!(v.forward_speed().abs() < 0.05);
    }

    #[test]
    fn full_throttle_pulls_away() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        settle(&mut v, &ground, Controls::default());
        let speed = driven(
            &mut v,
            &ground,
            Controls {
                throttle: 1.0,
                ..Default::default()
            },
            3.0,
        );
        assert!(speed > 8.0, "3 s: {speed} m/s");
        assert_eq!(v.gearbox.gear >= 1, true);
    }

    #[test]
    fn the_engine_reaches_a_sane_top_speed() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        let controls = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        settle(&mut v, &ground, controls);
        let speed = driven(&mut v, &ground, controls, 30.0);
        assert!(speed > 25.0, "{speed}");
        assert!(speed <= MAX_SPEED + 1.0, "{speed}");
    }

    #[test]
    fn braking_stops_from_twenty() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        let controls = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        settle(&mut v, &ground, Controls::default());
        driven(&mut v, &ground, controls, 5.0);
        let brake = Controls {
            brake: 1.0,
            ..Default::default()
        };
        let start = v.origin();
        driven(&mut v, &ground, brake, 3.0);
        let stop = ((v.origin()[0] - start[0]).powi(2) + (v.origin()[2] - start[2]).powi(2)).sqrt();
        assert!(v.forward_speed() < 1.0, "still {}", v.forward_speed());
        assert!(stop > 5.0 && stop < 80.0, "stopped in {stop} m");
    }

    #[test]
    fn steering_turns_the_car() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        settle(&mut v, &ground, Controls::default());
        let controls = Controls {
            throttle: 0.8,
            steer: 1.0,
            ..Default::default()
        };
        let yaw_before = v.yaw();
        driven(&mut v, &ground, controls, 3.0);
        let turned = normalize(v.yaw() - yaw_before);
        assert!(turned.abs() > 0.2, "turned {turned} rad");
    }

    #[test]
    fn the_handbrake_slides_the_rear() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        let controls = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        settle(&mut v, &ground, controls);
        driven(&mut v, &ground, controls, 4.0);
        let before = v.yaw();
        let slide = Controls {
            throttle: 0.2,
            steer: 0.7,
            handbrake: true,
            ..Default::default()
        };
        driven(&mut v, &ground, slide, 1.5);
        assert!(normalize(v.yaw() - before).abs() > 0.15);
        assert!(v.slip > 0.05, "slip {}", v.slip);
    }

    #[test]
    fn a_wheel_over_a_drop_does_not_throw_the_body() {
        let mut v = car();
        let ground = Ground {
            surfaces: [Some(0.0), Some(0.0), Some(-3.0), Some(0.0)],
            friction: [1.0; 4],
        };
        settle(&mut v, &ground, Controls::default());
        assert!(v.roll().abs() < 0.35, "roll {}", v.roll());
        assert!(v.pitch().abs() < 0.35, "pitch {}", v.pitch());
    }

    #[test]
    fn ten_thousand_random_steps_stay_finite() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f64 / (1u64 << 24) as f64
        };
        for _ in 0..10_000 {
            let controls = Controls {
                throttle: next(),
                brake: next() * 0.5,
                steer: next() * 2.0 - 1.0,
                handbrake: next() > 0.9,
            };
            v.step(&controls, &ground, STEP);
            assert!(v.body.pos.is_finite() && v.body.vel.is_finite());
            assert!(v.yaw().is_finite() && v.pitch().is_finite() && v.roll().is_finite());
        }
    }
    #[test]
    fn nought_to_a_hundred_is_in_a_car_s_range() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        settle(&mut v, &ground, Controls::default());
        let controls = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        let mut seconds = 0.0;
        while v.forward_speed() * 3.6 < 100.0 && seconds < 20.0 {
            for _ in 0..SUBSTEPS {
                v.step(&controls, &ground, STEP);
            }
            seconds += 0.05;
        }
        assert!((4.0..10.0).contains(&seconds), "0-100 in {seconds} s");
    }

    #[test]
    fn a_sweeping_corner_holds_the_road_without_rolling_over() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        settle(&mut v, &ground, Controls::default());
        let gas = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        while v.forward_speed() * 3.6 < 60.0 {
            for _ in 0..SUBSTEPS {
                v.step(&gas, &ground, STEP);
            }
        }
        let before = v.yaw();
        let mut roll: f64 = 0.0;
        let mut grip: f64 = 0.0;
        let turn = Controls {
            throttle: 0.5,
            steer: 0.35,
            ..Default::default()
        };
        for _ in 0..(60 * SUBSTEPS) {
            let last = v.body.vel;
            v.step(&turn, &ground, STEP);
            roll = roll.max(v.roll().abs());
            grip = grip.max((v.body.vel - last).length() / STEP);
        }
        let turned = normalize(v.yaw() - before).abs();
        assert!(turned > 0.7, "turned {turned} rad");
        assert!(grip > 6.0, "only {grip} m/s^2 of grip");
        assert!(roll < 0.20, "rolled {roll} rad");
    }

    #[test]
    fn the_body_dives_under_braking_and_squats_under_power() {
        let mut v = car();
        let ground = Ground::flat(0.0);
        settle(&mut v, &ground, Controls::default());
        let base = v.pitch();
        let gas = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        let mut squat: f64 = 0.0;
        for _ in 0..(20 * SUBSTEPS) {
            v.step(&gas, &ground, STEP);
            squat = squat.min(v.pitch() - base);
        }
        // Nose up under power is a negative pitch.
        assert!(squat < -0.005, "squat {squat}");
        let brake = Controls {
            brake: 1.0,
            ..Default::default()
        };
        let before = v.pitch();
        let mut dive: f64 = 0.0;
        for _ in 0..(20 * SUBSTEPS) {
            v.step(&brake, &ground, STEP);
            dive = dive.max(v.pitch() - before);
        }
        assert!(dive > 0.005, "dive {dive}");
    }
}

#[cfg(test)]
mod servo_test {
    use super::*;

    fn run(pin_every_tick: bool) -> (f64, bool, f64) {
        let ground = Ground::flat(0.0);
        let mut v = Vehicle::new([0.0, 0.0, 0.0], 0.0);
        let mut entity = v.origin();
        let gas = Controls {
            throttle: 1.0,
            ..Default::default()
        };
        let mut airborne_ticks = 0;
        for tick in 0..(30 * 20) {
            if pin_every_tick {
                v.set_origin(entity);
            }
            for _ in 0..SUBSTEPS {
                v.step(&gas, &ground, STEP);
            }
            let target = v.origin();
            entity = [
                entity[0] + (target[0] - entity[0]).clamp(-2.5, 2.5),
                entity[1] + (target[1] - entity[1]).clamp(-0.3, 0.3),
                entity[2] + (target[2] - entity[2]).clamp(-2.5, 2.5),
            ];
            if v.airborne {
                airborne_ticks += 1;
            }
            if tick == 30 * 20 - 1 {
                return (v.forward_speed() * 3.6, v.airborne, entity[1]);
            }
        }
        (v.forward_speed() * 3.6, v.airborne, entity[1])
    }

    #[test]
    fn compare_pinning() {
        let (pinned_speed, pinned_air, pinned_y) = run(true);
        let (free_speed, free_air, free_y) = run(false);
        println!("pinned: {pinned_speed:.1} km/h airborne={pinned_air} y={pinned_y:.2}");
        println!("free:   {free_speed:.1} km/h airborne={free_air} y={free_y:.2}");
    }
}

#[cfg(test)]
mod snow_trace {
    use super::*;

    #[test]
    fn trace_snow() {
        for mu in [0.35, 0.5, 1.0] {
            let ground = Ground {
                surfaces: [Some(0.0); 4],
                friction: [mu; 4],
            };
            let mut v = Vehicle::new([0.0, 0.0, 0.0], 0.0);
            for _ in 0..(80 * SUBSTEPS) {
                v.step(&Controls::default(), &ground, STEP);
            }
            let gas = Controls {
                throttle: 1.0,
                ..Default::default()
            };
            let mut t = 0.0;
            let mut air = 0;
            let mut next_print = 0.0;
            for _ in 0..(30 * 20 * SUBSTEPS) {
                v.step(&gas, &ground, STEP);
                t += STEP;
                if v.airborne {
                    air += 1;
                }
                if t >= next_print {
                    next_print += 1.0;
                    println!(
                        "  mu={mu} t={t:.0} v={:.1} yaw={:.1} steer={:.2} om={:?} slip={:.2}",
                        v.forward_speed() * 3.6,
                        v.yaw().to_degrees(),
                        v.steer,
                        v.wheels.each_ref().map(|w| w.omega.round()),
                        v.slip
                    );
                }
            }
            let turned = {
                let before = v.yaw();
                let turn = Controls {
                    throttle: 0.6,
                    steer: 0.6,
                    ..Default::default()
                };
                for _ in 0..(60 * SUBSTEPS) {
                    v.step(&turn, &ground, STEP);
                }
                normalize(v.yaw() - before).to_degrees()
            };
            println!(
                "mu={mu}: 30s {:.1} km/h gear {} rpm {:.0} airborne {:.2}% turned {:.0} deg",
                v.forward_speed() * 3.6,
                v.gearbox.gear,
                v.gearbox.rpm,
                100.0 * f64::from(air) / (30.0 * 20.0 * SUBSTEPS as f64),
                turned
            );
        }
    }
}
