//! Sampling the world under the car: the height and friction of each wheel's
//! ground, and whether the body's footprint has run into a wall.

use horizon_client::mod_api::Host;

use super::Frame;
use super::tuning::{self, TRACK};

/// How far below the car the ground is still looked for, in blocks.
const DROP: i32 = 4;
/// How far above it a wheel may already be.
const RISE: i32 = 1;

/// The car's four wheels in local coordinates: forward offset and side offset
/// (positive is right).
fn wheels() -> [(f64, f64); 4] {
    let half = TRACK * 0.5;
    [
        (tuning::A, -half),
        (tuning::A, half),
        (-tuning::B, -half),
        (-tuning::B, half),
    ]
}

/// The unit forward and right vectors for `yaw` (radians).
fn axes(yaw: f64) -> ([f64; 2], [f64; 2]) {
    let forward = [-yaw.sin(), yaw.cos()];
    let right = [-yaw.cos(), -yaw.sin()];
    (forward, right)
}

/// The ground under `pos` for a car facing `yaw`.
pub fn sample(host: &dyn Host, pos: [f64; 3], yaw: f64) -> Frame {
    let (forward, right) = axes(yaw);
    let mut heights = [f64::NAN; 4];
    let mut friction = [1.0; 4];
    for (i, (along, side)) in wheels().iter().copied().enumerate() {
        let x = pos[0] + forward[0] * along + right[0] * side;
        let z = pos[2] + forward[1] * along + right[1] * side;
        if let Some((top, block)) = ground_at(host, x, z, pos[1]) {
            heights[i] = top;
            friction[i] = tuning::friction_of(&block);
        }
    }
    // A wheel hanging over a drop must not drag the car down: the mean of
    // four samples follows one cliff. Sit on the middle two instead.
    let mut known: Vec<f64> = heights.iter().copied().filter(|h| h.is_finite()).collect();
    known.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let target_y = match known.len() {
        0 => pos[1],
        1 => known[0],
        2 => (known[0] + known[1]) * 0.5,
        _ => {
            let mid = known.len() / 2;
            (known[mid - 1] + known[mid]) * 0.5
        }
    };
    Frame {
        heights,
        friction,
        target_y,
        blocked: blocked(host, pos, yaw),
    }
}

/// The top of the ground under `(x, z)` at or below `from_y + RISE`.
pub fn surface(host: &dyn Host, x: f64, z: f64, from_y: f64) -> Option<f64> {
    ground_at(host, x, z, from_y).map(|(top, _)| top)
}

/// Whether `point` is inside a block's collision box.
pub fn solid(host: &dyn Host, point: [f64; 3]) -> bool {
    solid_at(host, point[0], point[1], point[2])
}

/// The top of the highest collision box under `(x, z)` at or below `from_y`
/// and the block it belongs to.
fn ground_at(host: &dyn Host, x: f64, z: f64, from_y: f64) -> Option<(f64, String)> {
    let (min_y, height) = host.world_height();
    let max_y = min_y + height.max(1) - 1;
    let bx = x.floor() as i32;
    let bz = z.floor() as i32;
    let start = ((from_y + f64::from(RISE)).floor() as i32).clamp(min_y, max_y);
    let stop = ((from_y.floor() as i32) - DROP).max(min_y);
    if start < stop {
        return None;
    }
    let (lx, lz) = (x - f64::from(bx), z - f64::from(bz));
    // A wheel may step up by RISE, never onto a block above that: otherwise a
    // ceiling under the same column would read as the ground.
    let limit = from_y + f64::from(RISE) + 0.05;
    for by in (stop..=start).rev() {
        let mut top: Option<f64> = None;
        for b in host.block_collision([bx, by, bz]) {
            let [x0, y0, z0, x1, y1, z1] = b;
            let inside =
                lx > x0 - 1e-7 && lx < x1 + 1e-7 && lz > z0 - 1e-7 && lz < z1 + 1e-7 && y1 > y0;
            if !inside {
                continue;
            }
            let surface = f64::from(by) + y1;
            if surface <= limit {
                top = Some(top.map_or(surface, |t: f64| t.max(surface)));
            }
        }
        if let Some(top) = top {
            return Some((top, host.block([bx, by, bz])));
        }
    }
    None
}

/// Whether the body's footprint overlaps a block this tick.
pub fn blocked(host: &dyn Host, pos: [f64; 3], yaw: f64) -> bool {
    // The body is about 3.9 blocks long and 1.85 wide; the wheels reach a
    // little further out than the panels.
    const CORNERS: [(f64, f64); 6] = [
        (1.82, -0.88),
        (1.82, 0.88),
        (-1.82, -0.88),
        (-1.82, 0.88),
        (1.95, 0.0),
        (-1.95, 0.0),
    ];
    let (forward, right) = axes(yaw);
    for (along, side) in CORNERS {
        let x = pos[0] + forward[0] * along + right[0] * side;
        let z = pos[2] + forward[1] * along + right[1] * side;
        if solid_at(host, x, pos[1] + 0.35, z) || solid_at(host, x, pos[1] + 1.0, z) {
            return true;
        }
    }
    false
}

/// Whether `point` is strictly inside a collision box of its block.
fn solid_at(host: &dyn Host, x: f64, y: f64, z: f64) -> bool {
    let block = [x.floor() as i32, y.floor() as i32, z.floor() as i32];
    let (lx, ly, lz) = (
        x - f64::from(block[0]),
        y - f64::from(block[1]),
        z - f64::from(block[2]),
    );
    for b in host.block_collision(block) {
        let [x0, y0, z0, x1, y1, z1] = b;
        if lx > x0 + 1e-3
            && lx < x1 - 1e-3
            && ly > y0 + 1e-3
            && ly < y1 - 1e-3
            && lz > z0 + 1e-3
            && lz < z1 - 1e-3
        {
            return true;
        }
    }
    false
}
