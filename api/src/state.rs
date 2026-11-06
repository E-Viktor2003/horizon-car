//! The mod's process-wide state: cars, input, queued actions and the
//! snapshots the client renders.
//!
//! Single player and a LAN host share one process: the integrated server
//! writes [`SNAPSHOTS`] and the window's client reads them. A LAN guest gets
//! the same snapshots over `horizon_car:state` (see [`crate::net`]).

use std::sync::Mutex;

use horizon_client::mod_api::Host;

use crate::physics::{self, ground, tuning, Controls, Vehicle};

pub const CAR_TAG: &str = "horizon_car";
pub const CAR_NAME: &str = "horizon_car";
pub const MAX_CARS: usize = 8;

/// The height of a pig's saddle above its feet (`getPassengerAttachmentPoint`).
pub const SEAT_HEIGHT: f64 = 0.868_75;

/// Who is driving a car this tick. The host has no UUID in the server hooks
/// that `Host::player` serves, so it is its own case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Driver {
    Host,
    Guest(u128),
}

pub struct Car {
    pub id: u32,
    pub dimension: String,
    pub vehicle: Vehicle,
    /// The state tag currently on the entity, to replace instead of piling up.
    pub saved_tag: String,
    pub save_timer: f64,
    pub brake_light: bool,
}

/// A pose the client interpolates between server ticks.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
    pub pos: [f64; 3],
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
    pub spin_f: f64,
    pub spin_r: f64,
    pub steer: f64,
}

impl Pose {
    pub fn of(vehicle: &Vehicle) -> Self {
        Self {
            pos: vehicle.pos,
            yaw: vehicle.yaw,
            pitch: vehicle.pitch,
            roll: vehicle.roll,
            spin_f: vehicle.spin_f,
            spin_r: vehicle.spin_r,
            steer: vehicle.steer,
        }
    }

    pub fn lerp(&self, next: &Pose, t: f64) -> Pose {
        let t = t.clamp(0.0, 1.0);
        let mix = |a: f64, b: f64| a + (b - a) * t;
        // Wheels wrap every turn; take the short way round.
        let spin = |a: f64, b: f64| physics::normalize(b - a) * t + a;
        Pose {
            pos: [
                mix(self.pos[0], next.pos[0]),
                mix(self.pos[1], next.pos[1]),
                mix(self.pos[2], next.pos[2]),
            ],
            yaw: physics::normalize(mix(self.yaw, next.yaw)),
            pitch: mix(self.pitch, next.pitch),
            roll: mix(self.roll, next.roll),
            spin_f: physics::normalize(spin(self.spin_f, next.spin_f)),
            spin_r: physics::normalize(spin(self.spin_r, next.spin_r)),
            steer: mix(self.steer, next.steer),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Snapshot {
    pub id: u32,
    pub prev: Pose,
    pub cur: Pose,
    pub speed: f64,
    pub gear: i32,
    pub rpm: f64,
    pub flags: u8,
}

impl Snapshot {
    pub const BRAKE: u8 = 1;
    pub const REVERSE: u8 = 2;
    pub const HANDBRAKE: u8 = 4;
    pub const ENGINE: u8 = 8;

    pub fn pose(&self, partial: f32) -> Pose {
        self.prev.lerp(&self.cur, f64::from(partial))
    }

    pub fn braking(&self) -> bool {
        self.flags & Self::BRAKE != 0
    }
}

pub enum Action {
    Spawn {
        dimension: String,
        pos: [f64; 3],
        yaw: f64,
    },
    Remove {
        id: u32,
    },
}

static CARS: Mutex<Vec<Car>> = Mutex::new(Vec::new());
static SNAPSHOTS: Mutex<Vec<Snapshot>> = Mutex::new(Vec::new());
static PENDING: Mutex<Vec<Action>> = Mutex::new(Vec::new());
static CHECK_JOIN: Mutex<Vec<(u32, String)>> = Mutex::new(Vec::new());
static KEYS: Mutex<u32> = Mutex::new(0);
static GUEST_INPUT: Mutex<Vec<(u128, i32)>> = Mutex::new(Vec::new());

pub const KEY_FORWARD: u32 = 1;
pub const KEY_BACK: u32 = 2;
pub const KEY_LEFT: u32 = 4;
pub const KEY_RIGHT: u32 = 8;
pub const KEY_HANDBRAKE: u32 = 16;

fn lock<T>(cell: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    cell.lock().unwrap_or_else(|e| e.into_inner())
}

/// A keyboard key the mod watches. Physical keys, as vanilla binds them.
pub fn key_bit(name: &str) -> Option<u32> {
    Some(match name {
        "key.keyboard.w" => KEY_FORWARD,
        "key.keyboard.s" => KEY_BACK,
        "key.keyboard.a" => KEY_LEFT,
        "key.keyboard.d" => KEY_RIGHT,
        "key.keyboard.space" => KEY_HANDBRAKE,
        _ => return None,
    })
}

pub fn set_key(bit: u32, pressed: bool) {
    let mut keys = lock(&KEYS);
    if pressed {
        *keys |= bit;
    } else {
        *keys &= !bit;
    }
}

pub fn clear_keys() {
    *lock(&KEYS) = 0;
}

pub fn set_guest_input(uuid: u128, flags: i32) {
    let mut input = lock(&GUEST_INPUT);
    if let Some(entry) = input.iter_mut().find(|(id, _)| *id == uuid) {
        entry.1 = flags;
    } else {
        input.push((uuid, flags));
    }
}

pub fn queue(action: Action) {
    lock(&PENDING).push(action);
}

pub fn car_count() -> usize {
    lock(&CARS).len()
}

/// The snapshot of the car the player at `pos` is sitting in.
pub fn seated_in(pos: [f64; 3]) -> Option<Snapshot> {
    lock(&SNAPSHOTS)
        .iter()
        .copied()
        .find(|s| seated(pos, s.cur.pos))
}

/// Whether a player's feet are on a car's saddle.
pub fn seated(player: [f64; 3], car: [f64; 3]) -> bool {
    seated_within(player, car, 2.5, 0.7)
}

/// The strict test for the server tick, where both positions are exact.
pub fn seated_exact(player: [f64; 3], car: [f64; 3]) -> bool {
    seated_within(player, car, 0.55, 0.5)
}

fn seated_within(player: [f64; 3], car: [f64; 3], reach: f64, vertical: f64) -> bool {
    let dx = player[0] - car[0];
    let dz = player[2] - car[2];
    let dy = player[1] - car[1];
    dx * dx + dz * dz < reach * reach && (dy - SEAT_HEIGHT).abs() < vertical
}

pub fn snapshot(id: u32) -> Option<Snapshot> {
    lock(&SNAPSHOTS).iter().find(|s| s.id == id).copied()
}

pub fn snapshots() -> Vec<Snapshot> {
    lock(&SNAPSHOTS).clone()
}

/// Replace the client's snapshots with the server's (a LAN guest).
pub fn set_snapshots(list: Vec<Snapshot>) {
    *lock(&SNAPSHOTS) = list;
}

/// Called once per server tick for one level, before that level ticks.
pub fn tick(host: &mut dyn Host, dimension: &str) {
    process_actions(host);
    process_join_checks(host);

    let ids: Vec<u32> = {
        let cars = lock(&CARS);
        cars.iter()
            .filter(|c| c.dimension == dimension)
            .map(|c| c.id)
            .collect()
    };

    let mut tag_updates: Vec<(u32, String, String)> = Vec::new();
    {
        let mut cars = lock(&CARS);
        for id in ids {
            let Some(index) = cars.iter().position(|c| c.id == id) else {
                continue;
            };
            if let Some(update) = tick_car(host, &mut cars[index]) {
                tag_updates.push(update);
            }
        }
    }

    // Tag writes can wake hooks; keep them outside the car lock.
    for (id, old, new) in tag_updates {
        if let Some(entity) = host.entity(id) {
            if !old.is_empty() {
                host.entity_remove_tag(entity, &old);
            }
            host.entity_add_tag(entity, &new);
        }
    }

    crate::net::broadcast(host);
}

/// Physics for one car. Returns the state tag to write when one is due.
fn tick_car(host: &mut dyn Host, car: &mut Car) -> Option<(u32, String, String)> {
    let Some(entity) = host.entity(car.id) else {
        // The chunk isn't loaded (or the entity is gone): keep the car and
        // skip its physics until it comes back.
        return None;
    };

    // The entity is the authority on where the car is: whatever the game let
    // it do last tick is where the physics starts this tick.
    if let Some(actual) = host.entity_position(entity) {
        car.vehicle.pos = actual;
    }

    let driver = find_driver(host, car);
    let controls = match driver {
        Some(driver) => controls(host, driver),
        None => Controls {
            handbrake: true,
            ..Default::default()
        },
    };

    let mut frame = ground::sample(host, car.vehicle.pos, car.vehicle.yaw);
    frame.blocked = false;
    let before = car.vehicle.pos;
    let dt = 0.05 / 4.0;
    for _ in 0..4 {
        car.vehicle.step(&controls, &frame, dt);
    }

    // The pig's box is small and the car's nose is long: stop the body from
    // driving through a wall the pig would miss.
    if ground::blocked(host, car.vehicle.pos, car.vehicle.yaw) {
        if !ground::blocked(host, before, car.vehicle.yaw) {
            car.vehicle.pos[0] = before[0];
            car.vehicle.pos[2] = before[2];
        }
        car.vehicle.vel[0] = -car.vehicle.vel[0] * 0.15;
        car.vehicle.vel[2] = -car.vehicle.vel[2] * 0.15;
        car.vehicle.yaw_rate *= 0.4;
    }

    // A car with nobody in it stays put.
    if driver.is_none() && car.vehicle.speed < 0.6 {
        car.vehicle.vel[0] = 0.0;
        car.vehicle.vel[2] = 0.0;
        car.vehicle.yaw_rate = 0.0;
    }

    // Ask the pig to go where the physics went. `entity_set_velocity` takes
    // blocks a tick, so the error over one tick is the velocity to set; a
    // clamp keeps a teleport-sized jump from throwing the rider.
    let target = car.vehicle.pos;
    if let Some(actual) = host.entity_position(entity) {
        let grounded = frame.heights.iter().any(|h| h.is_finite());
        let target_y = if grounded {
            frame.target_y + 0.01
        } else {
            car.vehicle.pos[1]
        };
        let vx = (target[0] - actual[0]).clamp(-2.5, 2.5);
        let vz = (target[2] - actual[2]).clamp(-2.5, 2.5);
        let vy = (target_y - actual[1]).clamp(-0.6, 0.6);
        host.entity_set_velocity(entity, [vx, vy, vz]);
    }

    car.brake_light = controls.brake > 0.05 || controls.handbrake;

    // Publish the pose for the client.
    let pose = Pose::of(&car.vehicle);
    let mut flags = Snapshot::ENGINE;
    if car.brake_light {
        flags |= Snapshot::BRAKE;
    }
    if car.vehicle.gearbox.gear < 0 {
        flags |= Snapshot::REVERSE;
    }
    if controls.handbrake {
        flags |= Snapshot::HANDBRAKE;
    }
    {
        let mut snaps = lock(&SNAPSHOTS);
        if let Some(snap) = snaps.iter_mut().find(|s| s.id == car.id) {
            snap.prev = snap.cur;
            snap.cur = pose;
            snap.speed = car.vehicle.forward_speed();
            snap.gear = car.vehicle.gearbox.gear;
            snap.rpm = car.vehicle.gearbox.rpm;
            snap.flags = flags;
        } else {
            snaps.push(Snapshot {
                id: car.id,
                prev: pose,
                cur: pose,
                speed: car.vehicle.forward_speed(),
                gear: car.vehicle.gearbox.gear,
                rpm: car.vehicle.gearbox.rpm,
                flags,
            });
        }
    }

    // Save the yaw and gear once a second, or when they changed a lot.
    car.save_timer += 0.05;
    let tag = state_tag(&car.vehicle);
    if car.save_timer >= 1.0 || car.saved_tag.is_empty() {
        car.save_timer = 0.0;
        if car.saved_tag != tag {
            let old = std::mem::replace(&mut car.saved_tag, tag.clone());
            return Some((car.id, old, tag));
        }
    }
    None
}

/// The tag that carries a car's yaw and gear across a save.
pub fn state_tag(vehicle: &Vehicle) -> String {
    let centi = (vehicle.yaw.to_degrees().rem_euclid(360.0) * 100.0).round() as i64 % 36_000;
    format!("hc_s{centi:05}_{}", vehicle.gearbox.gear)
}

fn find_driver(host: &dyn Host, car: &Car) -> Option<Driver> {
    if let Some(player) = host.player() {
        if player.dimension == car.dimension.as_str()
            && seated_exact([player.x, player.y, player.z], car.vehicle.pos)
        {
            return Some(Driver::Host);
        }
    }
    for guest in host.server_players() {
        if guest.dimension == car.dimension && seated_exact(guest.position, car.vehicle.pos) {
            return Some(Driver::Guest(guest.uuid));
        }
    }
    None
}

fn controls(_host: &dyn Host, driver: Driver) -> Controls {
    let flags: u32 = match driver {
        Driver::Host => *lock(&KEYS),
        Driver::Guest(uuid) => {
            let input = lock(&GUEST_INPUT);
            input
                .iter()
                .find(|(id, _)| *id == uuid)
                .map(|(_, f)| *f as u32)
                .unwrap_or(0)
        }
    };
    Controls {
        throttle: f64::from(u8::from(flags & KEY_FORWARD != 0)),
        brake: f64::from(u8::from(flags & KEY_BACK != 0)),
        steer: f64::from(i8::from(flags & KEY_RIGHT != 0) - i8::from(flags & KEY_LEFT != 0)),
        handbrake: flags & KEY_HANDBRAKE != 0,
    }
}

fn process_actions(host: &mut dyn Host) {
    let actions: Vec<Action> = std::mem::take(&mut *lock(&PENDING));
    for action in actions {
        match action {
            Action::Spawn {
                dimension,
                pos,
                yaw,
            } => {
                crate::spawn::spawn(host, &dimension, pos, yaw);
            }
            Action::Remove { id } => {
                let dimension = lock(&CARS)
                    .iter()
                    .find(|c| c.id == id)
                    .map(|c| c.dimension.clone());
                if let Some(dimension) = dimension {
                    host.remove_mob(&dimension, id);
                    remove(id);
                }
            }
        }
    }
}

fn process_join_checks(host: &mut dyn Host) {
    let checks: Vec<(u32, String)> = std::mem::take(&mut *lock(&CHECK_JOIN));
    for (id, dimension) in checks {
        if has_car(id) {
            continue;
        }
        crate::spawn::restore(host, id, &dimension);
    }
}

pub fn parse_saved(tag: &str) -> (f64, i32) {
    let mut yaw = 0.0;
    let mut gear = 1;
    if let Some(rest) = tag.strip_prefix("hc_s") {
        let mut parts = rest.split('_');
        if let Some(deg) = parts.next().and_then(|v| v.parse::<f64>().ok()) {
            yaw = (deg / 100.0).to_radians();
        }
        if let Some(g) = parts.next().and_then(|v| v.parse::<i32>().ok()) {
            gear = g.clamp(-1, 6);
        }
    }
    (yaw, gear)
}

pub fn remove(id: u32) {
    lock(&CARS).retain(|c| c.id != id);
    lock(&SNAPSHOTS).retain(|s| s.id != id);
}

pub fn insert(car: Car) {
    lock(&CARS).push(car);
}

/// Give the client a pose before the first physics tick.
pub fn publish(id: u32) {
    let pose = lock(&CARS)
        .iter()
        .find(|c| c.id == id)
        .map(|c| Pose::of(&c.vehicle));
    let Some(pose) = pose else {
        return;
    };
    let mut snaps = lock(&SNAPSHOTS);
    let snap = Snapshot {
        id,
        prev: pose,
        cur: pose,
        speed: 0.0,
        gear: 1,
        rpm: tuning::IDLE_RPM,
        flags: Snapshot::ENGINE,
    };
    if let Some(existing) = snaps.iter_mut().find(|s| s.id == id) {
        *existing = snap;
    } else {
        snaps.push(snap);
    }
}

pub fn has_car(id: u32) -> bool {
    lock(&CARS).iter().any(|c| c.id == id)
}

pub fn note_join(id: u32, dimension: &str) {
    lock(&CHECK_JOIN).push((id, dimension.to_owned()));
}

/// The nearest car to `pos` within `reach`, with its id and position.
pub fn nearest_car(pos: [f64; 3], reach: f64) -> Option<(u32, [f64; 3])> {
    let cars = lock(&CARS);
    let mut best: Option<(f64, u32, [f64; 3])> = None;
    for car in cars.iter() {
        let dx = car.vehicle.pos[0] - pos[0];
        let dz = car.vehicle.pos[2] - pos[2];
        let d = dx * dx + dz * dz;
        if d <= reach * reach && best.as_ref().is_none_or(|(b, _, _)| d < *b) {
            best = Some((d, car.id, car.vehicle.pos));
        }
    }
    best.map(|(_, id, pos)| (id, pos))
}

/// Forget everything: a world is closing or the window left a server.
pub fn clear() {
    lock(&CARS).clear();
    lock(&SNAPSHOTS).clear();
    lock(&PENDING).clear();
    lock(&CHECK_JOIN).clear();
    lock(&GUEST_INPUT).clear();
    clear_keys();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(yaw_degrees: f64, gear: i32) -> Vehicle {
        let mut car = Vehicle::new([0.0; 3], yaw_degrees.to_radians());
        car.gearbox.gear = gear;
        car
    }

    #[test]
    fn a_state_tag_round_trips() {
        let tag = state_tag(&at(45.0, 3));
        let (yaw, gear) = parse_saved(&tag);
        assert!((yaw.to_degrees() - 45.0).abs() < 0.01, "{tag}");
        assert_eq!(gear, 3);
    }

    #[test]
    fn a_reverse_tag_round_trips() {
        let tag = state_tag(&at(-100.0, -1));
        let (yaw, gear) = parse_saved(&tag);
        // 260 degrees and -100 degrees are the same way round.
        let turned = yaw.to_degrees().rem_euclid(360.0);
        assert!((turned - 260.0).abs() < 0.01, "{tag} -> {turned}");
        assert_eq!(gear, -1);
    }

    #[test]
    fn a_tag_that_is_not_ours_falls_back_to_the_defaults() {
        assert_eq!(parse_saved("something_else"), (0.0, 1));
        assert_eq!(parse_saved(""), (0.0, 1));
    }

    #[test]
    fn the_server_seat_test_is_strict() {
        let car = [10.0, 20.0, 10.0];
        let seat = [10.0, 20.0 + SEAT_HEIGHT, 10.0];
        assert!(seated_exact(seat, car));
        assert!(seated(seat, car));
        // On the ground beside the car: not the driver.
        assert!(!seated_exact([10.3, 20.0, 10.0], car));
        // On the saddle but a block and a half ahead: not this car's driver.
        assert!(!seated_exact([11.5, 20.0 + SEAT_HEIGHT, 10.0], car));
    }
}

