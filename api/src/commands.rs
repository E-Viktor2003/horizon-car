//! `/car`: put a car in front of the player, remove one, or ask for help.
//!
//! The command is registered on the server and heard in `perform_command`, so
//! it works for the host and for LAN guests alike. `perform_command` hands
//! over the acting player with their position, which `player_command` cannot
//! do here: it runs before the server has swapped the player in. Anything that
//! changes the world is queued and run in the next level tick.

use horizon_client::mod_api::{
    Flow, Host, NativeCommand, NativeServerPlayer, PerformCommandEvent, RegisterCommandsEvent,
    Text,
};

use crate::state::{self, Action, MAX_CARS};

/// The names the clients' command trees learn.
pub fn register(event: &mut RegisterCommandsEvent<'_>) {
    event.commands.push(NativeCommand::new("car"));
    event.commands.push(NativeCommand::new("car remove"));
    event.commands.push(NativeCommand::new("car help"));
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
        "W газ, S тормоз/задний ход, A/D руль, Space ручник, Shift — выйти",
    ] {
        lines.push(line.to_owned());
    }
}
