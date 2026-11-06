//! Turning a position into a car entity: the pig "chassis", its NBT, and
//! putting one into the world.

use std::collections::BTreeMap;

use horizon_client::mod_api::{Compound, Host, Tag};

use crate::physics::{ground, Vehicle};
use crate::state::{self, Car, CAR_NAME, CAR_TAG, MAX_CARS};

/// Spawn a car at `pos`, snapped to the ground, facing `yaw` radians.
pub fn spawn(host: &mut dyn Host, dimension: &str, pos: [f64; 3], yaw: f64) -> Option<u32> {
    if state::car_count() >= MAX_CARS {
        return None;
    }
    let spot = find_spot(host, pos);
    let nbt = car_nbt(yaw);
    let id = host.summon(dimension, "minecraft:pig", spot, nbt)?;

    let mut vehicle = Vehicle::new(spot, yaw);
    let tag = state::state_tag(&vehicle);
    vehicle.gearbox.gear = 1;
    state::insert(Car {
        id,
        dimension: dimension.to_owned(),
        vehicle,
        saved_tag: tag,
        save_timer: 0.0,
        brake_light: false,
    });
    state::publish(id);
    Some(id)
}

/// A free column near `pos`: the ground surface with two blocks of headroom.
fn find_spot(host: &dyn Host, pos: [f64; 3]) -> [f64; 3] {
    let mut y = pos[1] + 2.0;
    let bottom = pos[1] - 6.0;
    while y > bottom {
        if let Some(top) = ground::surface(host, pos[0], pos[2], y) {
            let feet = top + 0.001;
            let clear = ![0.1, 1.1, 2.05]
                .iter()
                .any(|d| ground::solid(host, [pos[0], feet + d, pos[2]]));
            if clear {
                return [pos[0], feet, pos[2]];
            }
            y = top - 0.5;
        } else {
            y -= 1.0;
        }
    }
    pos
}

/// The NBT a car's pig gets: invisible vanilla model, no gravity, a saddle to
/// mount, no self-movement, and the tags that name it and save its state.
pub fn car_nbt(yaw: f64) -> Compound {
    let mut nbt = Compound::new();
    nbt.insert("CustomName".into(), Tag::String(CAR_NAME.into()));
    nbt.insert("CustomNameVisible".into(), Tag::Byte(0));
    // `aiStep` only runs with AI on, and `travel` is where the velocity the
    // mod sets is applied; a movement speed of zero keeps the pig from
    // walking on its own.
    nbt.insert("NoAI".into(), Tag::Byte(0));
    nbt.insert("NoGravity".into(), Tag::Byte(1));
    nbt.insert("Silent".into(), Tag::Byte(1));
    nbt.insert("Invulnerable".into(), Tag::Byte(1));
    nbt.insert("PersistenceRequired".into(), Tag::Byte(1));
    nbt.insert("Saddle".into(), Tag::Byte(1));
    nbt.insert("Health".into(), Tag::Float(20.0f32.to_bits()));
    nbt.insert(
        "Rotation".into(),
        Tag::List(vec![
            Tag::Float((yaw.to_degrees() as f32).to_bits()),
            Tag::Float(0.0f32.to_bits()),
        ]),
    );
    nbt.insert(
        "Tags".into(),
        Tag::List(vec![
            Tag::String(CAR_TAG.into()),
            Tag::String(state::state_tag(&Vehicle::new([0.0; 3], yaw)).as_str().into()),
        ]),
    );
    let mut attributes = BTreeMap::new();
    attributes.insert(
        "id".into(),
        Tag::String("minecraft:movement_speed".into()),
    );
    attributes.insert("base".into(), Tag::Double(0.0f64.to_bits()));
    nbt.insert(
        "attributes".into(),
        Tag::List(vec![Tag::Compound(attributes)]),
    );
    nbt
}

/// The pose of a car that has just come back from the save, or `None` if the
/// entity isn't one of ours.
pub fn restore(host: &mut dyn Host, id: u32, dimension: &str) -> bool {
    if state::has_car(id) {
        return true;
    }
    let Some(entity) = host.entity(id) else {
        return false;
    };
    let tags = host.entity_tags(entity).unwrap_or_default();
    if !tags.iter().any(|t| t == CAR_TAG) {
        return false;
    }
    let Some(position) = host.entity_position(entity) else {
        return false;
    };
    let saved = tags
        .iter()
        .find(|t| t.starts_with("hc_s"))
        .cloned()
        .unwrap_or_default();
    let (yaw, gear) = state::parse_saved(&saved);
    let mut vehicle = Vehicle::new(position, yaw);
    vehicle.gearbox.gear = gear;
    state::insert(Car {
        id,
        dimension: dimension.to_owned(),
        vehicle,
        saved_tag: saved,
        save_timer: 0.0,
        brake_light: false,
    });
    state::publish(id);
    true
}
