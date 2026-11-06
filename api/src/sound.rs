//! A simple engine loop for the local driver: a vanilla minecart-inside sound
//! whose pitch follows the revs. It is not a recording, but it gives the car
//! an audible speed and gear.

use std::sync::atomic::{AtomicU32, Ordering};

use horizon_client::mod_api::Host;

use crate::physics::tuning;
use crate::state;

/// The vanilla sound used as an engine.
const SOUND: &str = "minecraft:entity.minecart.inside";
/// Ticks between one short sound and the next.
const INTERVAL: u32 = 4;

static TICKS: AtomicU32 = AtomicU32::new(0);

/// Called on the window's game once per client tick.
pub fn tick(host: &mut dyn Host) {
    let Some(player) = host.player() else {
        return;
    };
    let Some(snap) = state::seated_in([player.x, player.y, player.z]) else {
        return;
    };
    let count = TICKS.fetch_add(1, Ordering::Relaxed);
    if !count.is_multiple_of(INTERVAL) {
        return;
    }
    let rev = (snap.rpm / tuning::REDLINE_RPM).clamp(0.15, 1.0);
    let pitch = 0.55 + rev as f32 * 1.15;
    let volume = 0.22 + (snap.speed.abs() as f32 * 0.012).min(0.4);
    host.play_local_sound(
        SOUND,
        [snap.cur.pos[0], snap.cur.pos[1] + 0.6, snap.cur.pos[2]],
        volume,
        pitch,
        0,
    );
}
