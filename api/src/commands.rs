//! `/car`: put a car in front of the player, remove one, or ask for help.
//!
//! The command is registered on the server and heard in `perform_command`, so
//! it works for the host and for LAN guests alike. `perform_command` hands
//! over the acting player with their position, which `player_command` cannot
//! do here: it runs before the server has swapped the player in. Anything that
//! changes the world is queued and run in the next level tick.

use horizon_client::mod_api::{
    Flow, Host, NativeCommand, NativeServerPlayer, PerformCommandEvent, RegisterCommandsEvent, Text,
};

use crate::state::{self, Action, MAX_CARS};

/// The names the clients' command trees learn.
pub fn register(event: &mut RegisterCommandsEvent<'_>) {
    event.commands.push(NativeCommand::new("car"));
    event.commands.push(NativeCommand::new("car remove"));
    event.commands.push(NativeCommand::new("car help"));
    event.commands.push(NativeCommand::new("car debug"));
}

pub fn perform(host: &mut dyn Host, event: &mut PerformCommandEvent<'_>) -> Flow {
    let line = event.command.trim();
    let (head, rest) = match line.split_once(char::is_whitespace) {
        Some((head, rest)) => (head, rest.trim()),
        None => (line, ""),
    };
    if head != "car" {
        return Flow::Continue;
    }
    let player = event.player;
    let mut lines: Vec<String> = Vec::new();
    match rest {
        "" => place(player, &mut lines),
        "remove" => remove(player, &mut lines),
        "help" => help(&mut lines),
        "debug" => debug(host, player, &mut lines),
        other => lines.push(format!(
            "Неизвестный аргумент «{other}». /car, /car remove, /car help"
        )),
    }
    for line in lines {
        host.tell(player.uuid, Text::literal(line));
    }
    Flow::Cancel
}

fn place(player: &NativeServerPlayer, lines: &mut Vec<String>) {
    if state::car_count() >= MAX_CARS {
        lines.push(format!("Уже {MAX_CARS} машин — больше нельзя."));
        return;
    }
    let yaw = f64::from(player.yaw).to_radians();
    // Minecraft's yaw: 0 is +z and +90 is -x, which is the physics' forward.
    let forward = [-yaw.sin(), yaw.cos()];
    let pos = [
        player.position[0] + forward[0] * 4.0,
        player.position[1],
        player.position[2] + forward[1] * 4.0,
    ];
    state::queue(Action::Spawn {
        dimension: player.dimension.clone(),
        pos,
        yaw,
    });
    lines.push("Машина поставлена. Правый клик — сесть.".to_owned());
}

fn remove(player: &NativeServerPlayer, lines: &mut Vec<String>) {
    match state::nearest_car(player.position, &player.dimension, 12.0) {
        Some((id, _)) => {
            state::queue(Action::Remove { id });
            lines.push("Убираю машину.".to_owned());
        }
        None => lines.push("Рядом нет машины (ищу в 12 блоках).".to_owned()),
    }
}

fn help(lines: &mut Vec<String>) {
    for line in [
        "/car — поставить машину перед собой",
        "/car remove — убрать ближайшую машину",
        "/car help — эта справка",
        "/car debug — что сервер видит про ближайшую машину",
        "W газ, S тормоз/задний ход, A/D руль, Space ручник, Shift — выйти",
    ] {
        lines.push(line.to_owned());
    }
}

/// What the server, the window and the guest input have for the nearest car.
fn debug(host: &mut dyn Host, player: &NativeServerPlayer, lines: &mut Vec<String>) {
    let players = host.server_players();
    lines.push(format!(
        "server_players={} window={} player={}",
        players.len(),
        host.window(),
        host.player().is_some()
    ));
    for p in &players {
        lines.push(format!(
            "  {} {} pos={:?} entity_id={}",
            p.name, p.dimension, p.position, p.entity_id
        ));
    }
    match state::nearest_car(player.position, &player.dimension, 8.0) {
        Some((id, pos)) => {
            lines.push(format!("nearest car {id} at {pos:?}"));
            lines.push(state::debug(id, player.uuid));
            let dx = player.position[0] - pos[0];
            let dy = player.position[1] - pos[1];
            let dz = player.position[2] - pos[2];
            lines.push(format!(
                "seat from player: dx={dx:.2} dy={dy:.2} dz={dz:.2}, seat height {:.2}",
                state::SEAT_HEIGHT
            ));
        }
        None => lines.push("no car within 8 blocks".to_owned()),
    }
}
