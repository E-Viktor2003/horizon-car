//! The driver's view: sit the camera in the cockpit instead of on the
//! vanilla pig seat, which is higher than the roof. In third person (F5) the
//! vanilla detached camera is left alone so the car can be seen from outside.

use horizon_client::mod_api::{CameraPoseEvent, Host};

use crate::state;

/// Height of the driver's eyes above the car's feet, in blocks.
const EYE: f64 = 1.30;
/// How far ahead of the car's centre the eyes sit.
const AHEAD: f64 = 0.15;
/// The vanilla eye height above the player's feet. In first person the
/// camera starts here; in third person it starts blocks behind, so this is
/// what tells the two views apart.
const PLAYER_EYE: f64 = 1.62;
/// How far the camera has to be from the eye to count as detached.
const DETACHED: f64 = 0.75;

pub fn pose(host: &dyn Host, event: &mut CameraPoseEvent<'_>) {
    let Some(player) = host.player() else {
        return;
    };
    let Some(snap) = state::seated_in([player.x, player.y, player.z]) else {
        return;
    };
    let eye = [player.x, player.y + PLAYER_EYE, player.z];
    let third = distance2(*event.position, eye).sqrt() > DETACHED;
    state::set_third_person(third);
    if third {
        return;
    }
    let pose = snap.pose(event.partial);
    let forward = [-pose.yaw.sin(), pose.yaw.cos()];
    let position = [
        pose.pos[0] + forward[0] * AHEAD,
        pose.pos[1] + EYE,
        pose.pos[2] + forward[1] * AHEAD,
    ];
    if position.iter().all(|v| v.is_finite()) {
        *event.position = position;
    }
}

fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    dx * dx + dy * dy + dz * dz
}
