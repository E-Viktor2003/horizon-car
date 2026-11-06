//! Drawing the car: the model's quads in place of the pig, with the wheels
//! turned and spinning and the body leaning.

mod model_data;

use horizon_client::mod_api::{Flow, Host, MobSubmitEvent};

use crate::physics::{self, tuning};
use crate::state;

/// One flat-shaded quad of the car model, in model space (feet at y = 0,
/// +z forward, +x left).
pub struct Quad {
    pub p: [[f32; 3]; 4],
    pub color: u32,
}

/// Where the body leans and pitches: the centre of mass, above the feet.
const PIVOT: [f32; 3] = [0.0, tuning::CG_HEIGHT as f32, 0.0];
/// The brake lights' colour in the generated model.
const BRAKE_LIGHT: u32 = 0xEB4141;
/// A lit brake light.
const BRAKE_GLOW: u32 = 0xFF_9A9A;

pub fn submit(host: &dyn Host, event: &mut MobSubmitEvent<'_>) -> Flow {
    if event.mob.custom_name != Some(state::CAR_NAME) {
        return Flow::Continue;
    }
    let Some(snap) = state::snapshot(event.mob.id) else {
        // The pose hasn't arrived yet: let vanilla draw for a moment.
        return Flow::Continue;
    };
    if let Some(player) = host.player()
        && state::seated([player.x, player.y, player.z], snap.cur.pos)
        && !state::third_person()
    {
        // The camera is inside the car: drawing the body would fill the
        // screen with its own panels. In third person the car stays visible.
        return Flow::Cancel;
    }

    let pose = snap.pose(event.mob.partial);
    let draw = &mut *event.draw;
    draw.push();
    // The game already posed the pig with its own body yaw; replace it with
    // the physics' yaw.
    let delta = physics::normalize(f64::from(event.mob.body_yaw).to_radians() - pose.yaw);
    draw.rotate_about([0.0, 1.0, 0.0], delta.to_degrees() as f32, [0.0; 3]);

    // The wheels stay planted; only the body leans on its suspension.
    let steer = -(pose.steer.to_degrees() as f32);
    wheel(
        host,
        draw,
        model_data::WHEEL_FL,
        model_data::PIVOT_FL,
        steer,
        pose.spin_f,
    );
    wheel(
        host,
        draw,
        model_data::WHEEL_FR,
        model_data::PIVOT_FR,
        steer,
        pose.spin_f,
    );
    wheel(
        host,
        draw,
        model_data::WHEEL_RL,
        model_data::PIVOT_RL,
        0.0,
        pose.spin_r,
    );
    wheel(
        host,
        draw,
        model_data::WHEEL_RR,
        model_data::PIVOT_RR,
        0.0,
        pose.spin_r,
    );

    // The body pitches and rolls about its centre of mass.
    draw.push();
    draw.rotate_about([1.0, 0.0, 0.0], pose.pitch.to_degrees() as f32, PIVOT);
    draw.rotate_about([0.0, 0.0, 1.0], pose.roll.to_degrees() as f32, PIVOT);
    let lit = snap.braking();
    for quad in model_data::BODY {
        let tint = if lit && quad.color == BRAKE_LIGHT {
            BRAKE_GLOW
        } else {
            quad.color
        };
        host.draw_quad(draw, model_data::SPRITE, quad.p, model_data::UV, tint);
    }
    draw.pop();

    draw.pop();
    Flow::Cancel
}

fn wheel(
    host: &dyn Host,
    draw: &mut horizon_client::mod_api::BlockEntityDraw,
    quads: &[Quad],
    pivot: [f32; 3],
    steer: f32,
    spin: f64,
) {
    // The generated quads are relative to the wheel's own pivot, so put the
    // pivot under the origin and turn there: that is the axle.
    draw.push();
    draw.translate(pivot);
    if steer != 0.0 {
        draw.rotate_about([0.0, 1.0, 0.0], steer, [0.0; 3]);
    }
    draw.rotate_about([1.0, 0.0, 0.0], spin.to_degrees() as f32, [0.0; 3]);
    for quad in quads {
        host.draw_quad(draw, model_data::SPRITE, quad.p, model_data::UV, quad.color);
    }
    draw.pop();
}
