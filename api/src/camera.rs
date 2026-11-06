//! The driver's view: sit the camera in the cockpit instead of on the
//! vanilla pig seat, which is higher than the roof.

use horizon_client::mod_api::{CameraPoseEvent, Host};

use crate::state;

/// Height of the driver's eyes above the car's feet, in blocks.
const EYE: f64 = 1.30;
/// How far ahead of the car's centre the eyes sit.
const AHEAD: f64 = 0.15;

pub fn pose(host: &dyn Host, event: &mut CameraPoseEvent<'_>) {
    let Some(player) = host.player() else {
        return;
    };
    let Some(snap) = state::seated_in([player.x, player.y, player.z]) else {
        return;
    };
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
