//! The mod's process-wide state: cars, input, queued actions and the
//! snapshots the client renders.
//!
//! Single player and a LAN host share one process: the integrated server
//! writes [`SNAPSHOTS`] and the window's client reads them. A LAN guest gets
//! the same snapshots over `horizon_car:state` (see [`crate::net`]).

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use horizon_client::mod_api::Host;

use crate::physics::{self, Controls, Vehicle, ground, tuning};

pub const CAR_TAG: &str = "horizon_car";
pub const CAR_NAME: &str = "horizon_car";
pub const MAX_CARS: usize = 8;

/// The height of a pig's saddle above its feet (`getPassengerAttachmentPoint`).
pub const SEAT_HEIGHT: f64 = 0.868_75;

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
    /// Angle of each wheel around its axle, FL, FR, RL, RR.
    pub spin: [f64; 4],
    /// Suspension compression of each wheel, in blocks.
    pub compression: [f64; 4],
    /// Steering angle of the two front wheels, radians, positive right.
    pub steer_l: f64,
    pub steer_r: f64,
    /// How far the body squats on its springs, in blocks.
    pub ride: f64,
}

impl Pose {
    pub fn of(vehicle: &Vehicle) -> Self {
        let wheel = |i: usize| &vehicle.wheels[i];
        Self {
            pos: vehicle.origin(),
            yaw: vehicle.yaw(),
            pitch: vehicle.pitch(),
            roll: vehicle.roll(),
            spin: std::array::from_fn(|i| wheel(i).spin),
            compression: std::array::from_fn(|i| wheel(i).compression),
            steer_l: wheel(0).steer,
            steer_r: wheel(1).steer,
            ride: vehicle.ride(),
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
            yaw: physics::normalize(self.yaw + physics::normalize(next.yaw - self.yaw) * t),
            pitch: mix(self.pitch, next.pitch),
            roll: mix(self.roll, next.roll),
            spin: std::array::from_fn(|i| physics::normalize(spin(self.spin[i], next.spin[i]))),
            compression: std::array::from_fn(|i| mix(self.compression[i], next.compression[i])),
            steer_l: mix(self.steer_l, next.steer_l),
            steer_r: mix(self.steer_r, next.steer_r),
            ride: mix(self.ride, next.ride),
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
/// Which entity rides which car, from the server's mount hook.
static RIDERS: Mutex<Vec<(u32, u32)>> = Mutex::new(Vec::new());
/// The window's own pedals, while its player sits in a car.
static LOCAL_DRIVE: Mutex<Option<(u32, Controls)>> = Mutex::new(None);
/// The last input packet of each LAN guest, by UUID.
static GUEST_INPUT: Mutex<Vec<(u128, i32)>> = Mutex::new(Vec::new());
/// The window is in the third-person view this frame: the camera hook must
/// leave vanilla's detached camera alone and the car must stay visible.
static THIRD_PERSON: AtomicBool = AtomicBool::new(false);

fn lock<T>(cell: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    cell.lock().unwrap_or_else(|e| e.into_inner())
}

/// The pedals from the local player's movement input: `forward` is W, `strafe`
/// is A (positive) and D (negative), `jump` is the handbrake.
pub fn controls_from_input(forward: f32, strafe: f32, jump: bool) -> Controls {
    Controls {
        throttle: f64::from(forward.max(0.0)),
        brake: f64::from((-forward).max(0.0)),
        steer: f64::from(-strafe).clamp(-1.0, 1.0),
        handbrake: jump,
    }
}

/// The pedals from a guest's `PlayerInput` flag bits.
pub fn controls_from_flags(flags: u32) -> Controls {
    Controls {
        throttle: f64::from(u8::from(flags & 1 != 0)),
        brake: f64::from(u8::from(flags & 2 != 0)),
        steer: f64::from(i8::from(flags & 8 != 0) - i8::from(flags & 4 != 0)),
        handbrake: flags & 16 != 0,
    }
}

pub fn set_local_drive(car: u32, controls: Controls) {
    *lock(&LOCAL_DRIVE) = Some((car, controls));
}

pub fn clear_local_drive() {
    *lock(&LOCAL_DRIVE) = None;
}

/// The local window's pedals, when its player is in `car`.
pub fn local_drive(car: u32) -> Option<Controls> {
    lock(&LOCAL_DRIVE)
        .filter(|(id, _)| *id == car)
        .map(|(_, controls)| controls)
}

pub fn set_rider(car: u32, rider: Option<u32>) {
    let mut riders = lock(&RIDERS);
    riders.retain(|(id, _)| *id != car);
    if let Some(rider) = rider {
        riders.push((car, rider));
    }
}

/// The entity riding `car`, from the mount hook.
pub fn rider(car: u32) -> Option<u32> {
    lock(&RIDERS)
        .iter()
        .find(|(id, _)| *id == car)
        .map(|(_, rider)| *rider)
}

pub fn set_third_person(on: bool) {
    THIRD_PERSON.store(on, Ordering::Relaxed);
}

pub fn third_person() -> bool {
    THIRD_PERSON.load(Ordering::Relaxed)
}

pub fn set_guest_input(uuid: u128, flags: i32) {
    let mut input = lock(&GUEST_INPUT);
    if let Some(entry) = input.iter_mut().find(|(id, _)| *id == uuid) {
        entry.1 = flags;
    } else {
        input.push((uuid, flags));
    }
}

/// A guest's pedals, if they sent any input this session.
pub fn guest_controls(uuid: u128) -> Option<Controls> {
    let input = lock(&GUEST_INPUT);
    input
        .iter()
        .find(|(id, _)| *id == uuid)
        .map(|(_, flags)| controls_from_flags(*flags as u32))
}

fn merge(into: &mut Controls, other: Controls) {
    into.throttle = into.throttle.max(other.throttle);
    into.brake = into.brake.max(other.brake);
    into.steer = (into.steer + other.steer).clamp(-1.0, 1.0);
    into.handbrake |= other.handbrake;
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
        car.vehicle.set_origin(actual);
    }

    // A rider entity that is gone (logged out, removed, chunk unloaded) is
    // no driver any more.
    if let Some(rider) = rider(car.id)
        && host.entity(rider).is_none()
    {
        set_rider(car.id, None);
    }
    // The window's pedals, a guest the server's player list has on the seat,
    // and the mount hook's record all say the car is taken. `entity_mount`
    // fires before the seat lands, so the record alone is enough for the
    // first tick; the inputs come from the client or the guest's packets.
    let local = local_drive(car.id);
    let rider = rider(car.id);
    let origin = car.vehicle.origin();
    let guest_seat = host
        .server_players()
        .iter()
        .find(|g| g.dimension == car.dimension && seated_exact(g.position, origin))
        .map(|g| g.uuid);
    let occupied = local.is_some() || rider.is_some() || guest_seat.is_some();
    let mut controls = Controls {
        handbrake: !occupied,
        ..Default::default()
    };
    if let Some(local) = local {
        merge(&mut controls, local);
    }
    if let Some(guest) = guest_seat.and_then(guest_controls) {
        merge(&mut controls, guest);
    }

    let ground = ground::sample(host, car.vehicle.hubs());
    let before = car.vehicle.origin();
    for _ in 0..physics::SUBSTEPS {
        car.vehicle.step(&controls, &ground, physics::STEP);
    }

    // The pig's box is small and the car's nose is long: stop the body from
    // driving through a wall the pig would miss.
    let after = car.vehicle.origin();
    if ground::blocked(host, after, car.vehicle.yaw()) {
        if !ground::blocked(host, before, car.vehicle.yaw()) {
            car.vehicle.set_origin(before);
        }
        // Damp, do not bounce: a car brushing a wall used to flip its speed
        // every other tick, and that read as the whole car shaking.
        car.vehicle.body.vel.x *= 0.15;
        car.vehicle.body.vel.z *= 0.15;
        car.vehicle.body.omega.y *= 0.4;
    }

    // A car with nobody in it stays put.
    if !occupied && car.vehicle.forward_speed().abs() < 0.6 {
        car.vehicle.body.vel = physics::vec::V3::ZERO;
        car.vehicle.body.omega = physics::vec::V3::ZERO;
    }

    // Ask the pig to go where the physics went. `entity_set_velocity` takes
    // blocks a tick, so the error over one tick is the velocity to set; a
    // clamp keeps a teleport-sized jump from throwing the rider.
    let target = car.vehicle.origin();
    if let Some(actual) = host.entity_position(entity) {
        let vx = (target[0] - actual[0]).clamp(-2.5, 2.5);
        let vz = (target[2] - actual[2]).clamp(-2.5, 2.5);
        let vy = (target[1] - actual[1]).clamp(-0.3, 0.3);
        host.entity_set_velocity(entity, [vx, vy, vz]);
    }

    car.brake_light = occupied && (controls.brake > 0.05 || controls.handbrake);

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
    let centi = (vehicle.yaw().to_degrees().rem_euclid(360.0) * 100.0).round() as i64 % 36_000;
    format!("hc_s{centi:05}_{}", vehicle.gearbox.gear)
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
    set_rider(id, None);
    if lock(&LOCAL_DRIVE).is_some_and(|(car, _)| car == id) {
        clear_local_drive();
    }
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
/// The nearest car to `pos` in `dimension` within `reach`, with its id and
/// position.
pub fn nearest_car(pos: [f64; 3], dimension: &str, reach: f64) -> Option<(u32, [f64; 3])> {
    let cars = lock(&CARS);
    let mut best: Option<(f64, u32, [f64; 3])> = None;
    for car in cars.iter() {
        if car.dimension != dimension {
            continue;
        }
        let origin = car.vehicle.origin();
        let dx = origin[0] - pos[0];
        let dz = origin[2] - pos[2];
        let d = dx * dx + dz * dz;
        if d <= reach * reach && best.as_ref().is_none_or(|(b, _, _)| d < *b) {
            best = Some((d, car.id, origin));
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
    lock(&RIDERS).clear();
    lock(&GUEST_INPUT).clear();
    clear_local_drive();
    set_third_person(false);
}

/// One line for `/car debug`: what the server has for `car` and what the
/// window and the guest input hold.
pub fn debug(car: u32, player: u128) -> String {
    format!(
        "car {car}: rider={:?} local={:?} guest={:?}",
        rider(car),
        local_drive(car),
        guest_controls(player)
    )
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

    #[test]
    fn the_window_pedals_map_to_w_s_a_d_space() {
        // W, D, space.
        let c = controls_from_input(1.0, -1.0, true);
        assert_eq!(c.throttle, 1.0);
        assert_eq!(c.brake, 0.0);
        assert_eq!(c.steer, 1.0);
        assert!(c.handbrake);
        // S, A.
        let c = controls_from_input(-1.0, 1.0, false);
        assert_eq!(c.throttle, 0.0);
        assert_eq!(c.brake, 1.0);
        assert_eq!(c.steer, -1.0);
        assert!(!c.handbrake);
    }

    #[test]
    fn the_guest_flag_bits_map_to_the_same_pedals() {
        // forward 1, backward 2, left 4, right 8, jump 16.
        let c = controls_from_flags(1 | 8 | 16);
        assert_eq!(c.throttle, 1.0);
        assert_eq!(c.brake, 0.0);
        assert_eq!(c.steer, 1.0);
        assert!(c.handbrake);
        let c = controls_from_flags(2 | 4);
        assert_eq!(c.brake, 1.0);
        assert_eq!(c.steer, -1.0);
        assert!(!c.handbrake);
    }

    #[test]
    fn two_input_sources_add_up_without_oversteering() {
        let mut c = controls_from_input(1.0, 0.0, false);
        merge(&mut c, controls_from_flags(1 | 8));
        assert_eq!(c.throttle, 1.0);
        assert_eq!(c.steer, 1.0);
    }

    fn pose_at(yaw: f64) -> Pose {
        Pose {
            pos: [0.0; 3],
            yaw,
            pitch: 0.0,
            roll: 0.0,
            spin: [0.0; 4],
            compression: [0.0; 4],
            steer_l: 0.0,
            steer_r: 0.0,
            ride: 0.0,
        }
    }

    #[test]
    fn a_pose_crossing_the_wrap_turns_the_short_way() {
        // Heading just short of south-west to just past it: the model must
        // turn a couple of degrees, not most of the way round.
        let from = pose_at(std::f64::consts::PI - 0.02);
        let to = pose_at(-std::f64::consts::PI + 0.02);
        let middle = from.lerp(&to, 0.5);
        let turned = physics::normalize(middle.yaw - from.yaw);
        assert!(turned.abs() < 0.05, "turned {turned} rad");
    }

    #[test]
    fn a_pose_between_ticks_is_smooth() {
        let from = pose_at(0.4);
        let to = pose_at(0.6);
        assert!((from.lerp(&to, 0.0).yaw - 0.4).abs() < 1e-9);
        assert!((from.lerp(&to, 1.0).yaw - 0.6).abs() < 1e-9);
        assert!((from.lerp(&to, 0.5).yaw - 0.5).abs() < 1e-9);
    }
}
