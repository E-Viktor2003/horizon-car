//! The driver's speedometer, gear and revs.

use horizon_client::mod_api::{Corner, Host, HudEvent};

use crate::state::{self, Snapshot};

const BACKGROUND: u32 = 0x9000_0000;
const INK: u32 = 0xFF_FFFF;
const DIM: u32 = 0x9A_A0A0;
const ACCENT: u32 = 0xFF_C24A;
const PADDING: f32 = 3.0;
const HEIGHT: f32 = 34.0;

pub fn draw(host: &mut dyn Host, event: &mut HudEvent<'_>) {
    let Some(player) = host.player() else {
        return;
    };
    let Some(snap) = state::seated_in([player.x, player.y, player.z]) else {
        return;
    };

    let kmh = (snap.speed.abs() * 3.6).round() as i32;
    let gear = match snap.gear {
        g if g < 0 => "R".to_owned(),
        0 => "N".to_owned(),
        g => g.to_string(),
    };
    let first = format!("{kmh:>3} km/h");
    let second = format!("{gear}   {:>4} rpm", snap.rpm.round() as i32);
    let width = host
        .text_width(&first)
        .max(host.text_width(&second))
        .max(84.0)
        + PADDING * 2.0;

    let (x, y) = event.corners.place(Corner::BottomLeft, width, HEIGHT);
    host.fill(event.batch, [x, y, width, HEIGHT], BACKGROUND);
    host.text(event.batch, &first, [x + PADDING, y + PADDING], INK, true);
    let color = if snap.flags & Snapshot::HANDBRAKE != 0 {
        ACCENT
    } else {
        DIM
    };
    host.text(
        event.batch,
        &second,
        [x + PADDING, y + PADDING + 12.0],
        color,
        true,
    );

    // A rev bar along the bottom of the box.
    let redline = crate::physics::tuning::REDLINE_RPM;
    let full = width - PADDING * 2.0;
    let filled = ((snap.rpm / redline).clamp(0.0, 1.0) * f64::from(full)) as f32;
    host.fill(
        event.batch,
        [x + PADDING, y + HEIGHT - 4.0, full, 2.0],
        0x40_000000,
    );
    if filled > 0.0 {
        host.fill(
            event.batch,
            [x + PADDING, y + HEIGHT - 4.0, filled, 2.0],
            if snap.rpm / redline > 0.85 { ACCENT } else { 0xAA55FF88 },
        );
    }

    if snap.braking() {
        host.fill(
            event.batch,
            [x + PADDING, y + PADDING + 24.0, 6.0, 4.0],
            0xFFFF3030,
        );
    }
}
