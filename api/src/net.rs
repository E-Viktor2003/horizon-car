//! Sending car poses from a LAN host to its guests.
//!
//! The car entity itself is a normal pig the server syncs, so guests already
//! see it. What they cannot see is the mod's pose (its yaw, roll, wheel spin),
//! which is why the host sends a small binary snapshot every tick.

use horizon_client::mod_api::{Host, NetFromServerEvent};

use crate::state::{self, Pose, Snapshot};

pub const CHANNEL: &str = "horizon_car:state";
const MAGIC: [u8; 4] = *b"HCAR";
const VERSION: u8 = 1;
/// id + two poses + speed, rpm, gear, flags.
const RECORD: usize = 4 + 9 * 4 * 2 + 4 + 4 + 1 + 1;

pub fn broadcast(host: &mut dyn Host) {
    let guests = host.server_players();
    if guests.is_empty() {
        return;
    }
    let snapshots = state::snapshots();
    let data = encode(&snapshots);
    for guest in guests {
        let _ = host.net_send_to_player(guest.uuid, CHANNEL, &data);
    }
}

pub fn receive(event: &NetFromServerEvent<'_>) {
    if event.channel != CHANNEL {
        return;
    }
    if let Some(list) = decode(event.data) {
        state::set_snapshots(list);
    }
}

fn encode(snapshots: &[Snapshot]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + snapshots.len() * RECORD);
    out.extend_from_slice(&MAGIC);
    out.push(VERSION);
    out.extend_from_slice(&(snapshots.len().min(u16::MAX as usize) as u16).to_le_bytes());
    for snap in snapshots.iter().take(u16::MAX as usize) {
        out.extend_from_slice(&snap.id.to_le_bytes());
        pose(&mut out, &snap.prev);
        pose(&mut out, &snap.cur);
        out.extend_from_slice(&(snap.speed as f32).to_le_bytes());
        out.extend_from_slice(&(snap.rpm as f32).to_le_bytes());
        out.push(snap.gear.clamp(-1, 6) as i8 as u8);
        out.push(snap.flags);
    }
    out
}

fn pose(out: &mut Vec<u8>, pose: &Pose) {
    for value in [
        pose.pos[0],
        pose.pos[1],
        pose.pos[2],
        pose.yaw,
        pose.pitch,
        pose.roll,
        pose.spin_f,
        pose.spin_r,
        pose.steer,
    ] {
        out.extend_from_slice(&(value as f32).to_le_bytes());
    }
}

fn decode(data: &[u8]) -> Option<Vec<Snapshot>> {
    if data.len() < 7 || data[0..4] != MAGIC || data[4] != VERSION {
        return None;
    }
    let count = u16::from_le_bytes([data[5], data[6]]) as usize;
    if data.len() < 7 + count * RECORD {
        return None;
    }
    let mut cursor = Cursor { data, at: 7 };
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        let id = cursor.u32()?;
        let prev = cursor.pose()?;
        let cur = cursor.pose()?;
        let speed = f64::from(cursor.f32()?);
        let rpm = f64::from(cursor.f32()?);
        let gear = i32::from(cursor.u8()? as i8);
        let flags = cursor.u8()?;
        out.push(Snapshot {
            id,
            prev,
            cur,
            speed,
            gear,
            rpm,
            flags,
        });
    }
    Some(out)
}

struct Cursor<'a> {
    data: &'a [u8],
    at: usize,
}

impl Cursor<'_> {
    fn take(&mut self, n: usize) -> Option<&[u8]> {
        let end = self.at.checked_add(n)?;
        let slice = self.data.get(self.at..end)?;
        self.at = end;
        Some(slice)
    }

    fn u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }

    fn u32(&mut self) -> Option<u32> {
        let b = self.take(4)?;
        Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Option<f32> {
        let b = self.take(4)?;
        Some(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn pose(&mut self) -> Option<Pose> {
        let mut values = [0.0f64; 9];
        for value in &mut values {
            *value = f64::from(self.f32()?);
        }
        Some(Pose {
            pos: [values[0], values[1], values[2]],
            yaw: values[3],
            pitch: values[4],
            roll: values[5],
            spin_f: values[6],
            spin_r: values[7],
            steer: values[8],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Snapshot {
        Snapshot {
            id: 42,
            prev: Pose {
                pos: [1.0, 2.0, 3.0],
                yaw: 0.25,
                pitch: -0.01,
                roll: 0.02,
                spin_f: 1.5,
                spin_r: 1.25,
                steer: 0.1,
            },
            cur: Pose {
                pos: [1.5, 2.0, 3.5],
                yaw: 0.3,
                pitch: 0.0,
                roll: -0.02,
                spin_f: 2.0,
                spin_r: 1.75,
                steer: 0.12,
            },
            speed: 12.5,
            gear: 3,
            rpm: 3200.0,
            flags: Snapshot::ENGINE | Snapshot::BRAKE,
        }
    }

    #[test]
    fn a_snapshot_survives_a_round_trip() {
        let list = vec![sample()];
        let decoded = decode(&encode(&list)).expect("decodes");
        assert_eq!(decoded.len(), 1);
        let a = &list[0];
        let b = &decoded[0];
        assert_eq!(a.id, b.id);
        assert_eq!(a.gear, b.gear);
        assert_eq!(a.flags, b.flags);
        assert!((a.speed - b.speed).abs() < 1e-4);
        assert!((a.cur.pos[2] - b.cur.pos[2]).abs() < 1e-4);
        assert!((a.prev.yaw - b.prev.yaw).abs() < 1e-4);
    }

    #[test]
    fn junk_is_ignored() {
        assert!(decode(b"nope").is_none());
        assert!(decode(b"HCAR\x01\x02\x00").is_none());
    }
}
