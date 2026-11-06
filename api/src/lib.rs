//! Horizon Car: a drivable car with server-side physics.
//!
//! This is a Horizon trait mod (`docs/native-mods.md`). It uses only catalog
//! hook points:
//!
//! - `level_tick_pre` runs the physics on the server, 20 times a second.
//! - `perform_command` hears `/car` on the server, with the acting player.
//! - `movement_input` reads the window's pedals; `player_input` a LAN guest's.
//! - `entity_mount` knows who got into which car.
//! - `mob_submit` draws the model over the pig that carries the car.
//! - `render_name_tag` hides the chassis' name tag.
//! - `hud` and `camera_pose` put the driver inside it.
//! - `net_from_server` carries a LAN host's poses to its guests.
//! - `entity_join_level` picks a car up again after a save.
//!
//! The physics lives in [`physics`] and is plain arithmetic, so `cargo test`
//! runs it without the game.

mod camera;
mod commands;
mod hud;
mod net;
mod physics;
mod render;
mod sound;
mod spawn;
mod state;

use horizon_client::mod_api::{
    ClientDisconnectingEvent, EntityJoinLevelEvent, EntityLeaveLevelEvent, EntityMountEvent, Flow,
    Hooks, Host, HudEvent, LevelTickPreEvent, MobSubmitEvent, MovementInputEvent,
    NetFromServerEvent, PerformCommandEvent, PlayerInputEvent, PlayerSubmitEvent,
    RegisterCommandsEvent, RenderNameTagEvent, TickEvent,
};

pub fn hooks() -> Box<dyn Hooks> {
    Box::new(Car)
}

struct Car;

impl Hooks for Car {
    /// One server level tick: physics, spawning and removing cars, state
    /// tags, and the state a LAN host sends its guests.
    fn level_tick_pre(&self, host: &mut dyn Host, event: &mut LevelTickPreEvent<'_>) {
        state::tick(host, event.dimension);
    }

    /// The `/car` names the clients' command trees learn.
    fn register_commands(&self, _host: &mut dyn Host, event: &mut RegisterCommandsEvent<'_>) {
        commands::register(event);
    }

    /// `/car` typed by any player, on the server. `perform_command` gives the
    /// acting player's position; `player_command` runs before the swap.
    fn perform_command(&self, host: &mut dyn Host, event: &mut PerformCommandEvent<'_>) -> Flow {
        commands::perform(host, event)
    }

    /// The window player's pedals, whatever keys they are bound to. While
    /// they sit in a car, this is what that car drives on.
    fn movement_input(&self, host: &dyn Host, event: &mut MovementInputEvent) {
        let Some(player) = host.player() else {
            state::clear_local_drive();
            return;
        };
        match state::seated_in([player.x, player.y, player.z]) {
            Some(snap) => state::set_local_drive(
                snap.id,
                state::controls_from_input(event.forward, event.strafe, event.jump),
            ),
            None => state::clear_local_drive(),
        }
    }

    /// A LAN guest's movement flags, which arrive on the host's server.
    fn player_input(&self, _host: &mut dyn Host, event: &mut PlayerInputEvent<'_>) {
        state::set_guest_input(event.player, event.flags);
    }

    /// Who got into (or out of) which car. The seat itself stays vanilla.
    fn entity_mount(&self, _host: &mut dyn Host, event: &mut EntityMountEvent<'_>) -> Flow {
        if state::has_car(event.vehicle) {
            state::set_rider(event.vehicle, event.mounting.then_some(event.entity));
        }
        Flow::Continue
    }

    /// A pig entering a level may be a car coming back from the save.
    fn entity_join_level(&self, host: &mut dyn Host, event: &mut EntityJoinLevelEvent<'_>) -> Flow {
        if event.kind != "minecraft:pig" || host.window() || state::has_car(event.entity) {
            return Flow::Continue;
        }
        // The entity is not always reachable from inside this hook.
        state::note_join(event.entity, event.dimension);
        Flow::Continue
    }

    /// A car's chunk unloaded or the pig was removed.
    fn entity_leave_level(&self, host: &mut dyn Host, event: &mut EntityLeaveLevelEvent<'_>) {
        if !host.window() && state::has_car(event.entity) {
            state::remove(event.entity);
        }
    }

    /// The client tick plays the engine sound.
    fn tick(&self, host: &mut dyn Host, _event: &mut TickEvent) {
        if host.window() {
            sound::tick(host);
        }
    }

    /// Replace the pig's model with the car.
    fn mob_submit(&self, host: &dyn Host, event: &mut MobSubmitEvent<'_>) -> Flow {
        render::submit(host, event)
    }

    /// The chassis is a mechanism, not a pet: no name above it.
    fn render_name_tag(&self, _host: &dyn Host, event: &mut RenderNameTagEvent<'_>) {
        if state::snapshot(event.entity_id).is_some() {
            event.visible = false;
        }
    }

    /// The driver sits inside the car; in third person their own model would
    /// poke through the roof, so leave it out for this window.
    fn player_submit(&self, host: &dyn Host, event: &mut PlayerSubmitEvent<'_>) -> Flow {
        if event.who.local
            && host
                .player()
                .is_some_and(|p| state::seated_in([p.x, p.y, p.z]).is_some())
        {
            return Flow::Cancel;
        }
        Flow::Continue
    }

    /// The driver's speedometer.
    fn hud(&self, host: &mut dyn Host, event: &mut HudEvent<'_>) {
        hud::draw(host, event);
    }

    /// The driver's eyes, inside the cockpit.
    fn camera_pose(
        &self,
        host: &dyn Host,
        event: &mut horizon_client::mod_api::CameraPoseEvent<'_>,
    ) {
        camera::pose(host, event);
    }

    /// Third person: pull back far enough to see a four-block car, not just
    /// the back of the driver's head.
    fn camera_distance(
        &self,
        host: &dyn Host,
        event: &mut horizon_client::mod_api::CameraDistanceEvent<'_>,
    ) {
        if host
            .player()
            .is_some_and(|p| state::seated_in([p.x, p.y, p.z]).is_some())
        {
            event.distance = event.distance.max(camera::THIRD_PERSON_DISTANCE);
        }
    }

    /// Car poses from a LAN host.
    fn net_from_server(&self, _host: &mut dyn Host, event: &mut NetFromServerEvent<'_>) {
        net::receive(event);
    }

    /// Leaving a world: the client's poses and the server's cars are gone.
    fn client_disconnecting(&self, _host: &dyn Host, _event: &mut ClientDisconnectingEvent) {
        state::clear();
    }
}
