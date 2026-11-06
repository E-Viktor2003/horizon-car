//! The engine and the automatic gearbox.

use super::tuning::{
    DRIVETRAIN_EFFICIENCY, FINAL_DRIVE, GEAR_RATIOS, IDLE_RPM, PEAK_TORQUE, REDLINE_RPM,
    REVERSE_RATIO, SHIFT_TIME, UPSHIFT_RPM, WHEEL_RADIUS,
};

/// Full-throttle torque at `rpm`, N m.
pub fn torque(rpm: f64) -> f64 {
    let x = (rpm - 3800.0) / 3600.0;
    PEAK_TORQUE * (0.55 + 0.45 * (1.0 - x * x).max(0.0))
}

/// What the gearbox asks the wheels to do this substep.
#[derive(Clone, Copy, Debug, Default)]
pub struct Drive {
    /// Force along the car's forward axis, N.
    pub force: f64,
    /// Braking input after the direction is decided (0..1).
    pub brake: f64,
    #[allow(dead_code)]
    pub reverse: bool,
}

#[derive(Clone, Debug)]
pub struct Gearbox {
    /// -1 reverse, 1..=6 forward.
    pub gear: i32,
    pub rpm: f64,
    shift_timer: f64,
    reverse_timer: f64,
    /// Whether S may engage reverse: released at a standstill. Braking from
    /// speed to a stop with S held does not reverse by itself.
    reverse_armed: bool,
}

impl Default for Gearbox {
    fn default() -> Self {
        Self {
            gear: 1,
            rpm: IDLE_RPM,
            shift_timer: 0.0,
            reverse_timer: 0.0,
            reverse_armed: true,
        }
    }
}

impl Gearbox {
    pub fn ratio(&self) -> f64 {
        if self.gear < 0 {
            REVERSE_RATIO
        } else {
            GEAR_RATIOS[self.gear.clamp(0, 6) as usize]
        }
    }

    /// The engine speed the wheels back-drive at, rpm.
    pub fn wheel_rpm(&self, forward_speed: f64) -> f64 {
        forward_speed.abs() / WHEEL_RADIUS * 60.0 / (std::f64::consts::TAU) * self.ratio()
            * FINAL_DRIVE
    }

    /// Pick a gear, turn the engine and return the force it makes.
    ///
    /// `throttle` is W and `brake` is S. S brakes while moving forward and
    /// engages reverse after a moment at a standstill; in reverse W brakes.
    pub fn update(&mut self, throttle: f64, brake: f64, forward_speed: f64, dt: f64) -> Drive {
        self.shift_timer = (self.shift_timer - dt).max(0.0);

        // S released while standing arms reverse; holding it from a stop
        // keeps braking instead of backing up.
        if forward_speed.abs() > 0.4 {
            self.reverse_armed = false;
        }
        if brake < 0.5 && forward_speed.abs() < 0.4 {
            self.reverse_armed = true;
        }

        // Reverse engages when S is held at a standstill and disengages on W.
        if self.gear >= 0 {
            if brake > 0.5 && forward_speed.abs() < 0.4 && throttle < 0.05 && self.reverse_armed
            {
                self.reverse_timer += dt;
                if self.reverse_timer > 0.35 {
                    self.gear = -1;
                    self.shift_timer = SHIFT_TIME;
                }
            } else {
                self.reverse_timer = 0.0;
            }
        } else if throttle > 0.2 {
            self.gear = 1;
            self.shift_timer = SHIFT_TIME;
            self.reverse_timer = 0.0;
        }

        let reverse = self.gear < 0;
        let pedal = if reverse { brake } else { throttle };

        // Automatic shifts, forward only and never during a shift.
        if !reverse && self.shift_timer <= 0.0 && self.gear > 0 {
            let rpm = self.wheel_rpm(forward_speed);
            if rpm > UPSHIFT_RPM && self.gear < 6 && pedal > 0.05 {
                self.gear += 1;
                self.shift_timer = SHIFT_TIME;
            } else if rpm < super::tuning::DOWNSHIFT_RPM && self.gear > 1 {
                self.gear -= 1;
                self.shift_timer = 0.25;
            }
        }

        let ratio = self.ratio();
        let mut rpm = self.wheel_rpm(forward_speed);
        if rpm < IDLE_RPM || self.shift_timer > 0.0 {
            // The torque converter slips; a shift drops the engine to idle.
            rpm = IDLE_RPM + pedal * 900.0;
        }
        self.rpm = rpm.clamp(IDLE_RPM, REDLINE_RPM);

        let mut force = 0.0;
        if self.shift_timer <= 0.0 && pedal > 0.0 && self.rpm < REDLINE_RPM {
            force = torque(self.rpm) * ratio * FINAL_DRIVE * DRIVETRAIN_EFFICIENCY / WHEEL_RADIUS
                * pedal;
        }
        if reverse {
            force = -force;
        }

        Drive {
            force,
            brake: if reverse { throttle } else { brake },
            reverse,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::tuning::GEAR_RATIOS;

    #[test]
    fn the_torque_curve_peaks_in_the_middle() {
        assert!(torque(3800.0) > torque(1000.0));
        assert!(torque(3800.0) > torque(6400.0));
    }

    #[test]
    fn a_standstill_pull_engages_first() {
        let mut g = Gearbox::default();
        let d = g.update(1.0, 0.0, 0.0, 0.05);
        assert_eq!(g.gear, 1);
        assert!(d.force > 0.0);
    }

    #[test]
    fn s_at_a_standstill_reverses_after_a_moment() {
        let mut g = Gearbox::default();
        for _ in 0..20 {
            g.update(0.0, 1.0, 0.0, 0.05);
        }
        assert_eq!(g.gear, -1);
        let d = g.update(0.0, 1.0, 0.0, 0.05);
        assert!(d.force < 0.0);
        assert!(d.reverse);
    }

    #[test]
    fn braking_down_from_speed_does_not_engage_reverse() {
        let mut g = Gearbox::default();
        // Brake from 20 m/s to a stop with S still held.
        let mut speed = 20.0;
        for _ in 0..120 {
            g.update(0.0, 1.0, speed, 0.05);
            speed = (speed - 11.0 * 0.05).max(0.0);
        }
        assert_eq!(g.gear, 1);
    }

    #[test]
    fn high_revs_upshift() {
        let mut g = Gearbox::default();
        // 6000 engine rpm in first: v = rpm/ratio/final * wheel.
        let v = 6000.0 / GEAR_RATIOS[1] / FINAL_DRIVE * WHEEL_RADIUS * std::f64::consts::TAU / 60.0;
        g.update(1.0, 0.0, v, 0.05);
        assert_eq!(g.gear, 2);
    }

    #[test]
    fn a_shift_interrupts_the_pull() {
        let mut g = Gearbox::default();
        g.shift_timer = 0.2;
        let d = g.update(1.0, 0.0, 0.0, 0.05);
        assert_eq!(d.force, 0.0);
    }
}
