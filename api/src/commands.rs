//! `/car`: put a car in front of the player, remove one, or ask for help.
//!
//! The command is registered on the server and heard in `player_command`, so
//! it works for the host and for LAN guests alike. Anything that changes the
//! world is queued and run in the next level tick.

use horizon_client::mod_api::{
    Flow, Host, NativeCommand, PlayerCommandEvent, RegisterCommandsEvent, Text,
};

use crate::state::{self, Action, MAX_CARS};

/// The names the client's command tree learns.
pub fn register(event: &mut RegisterCommandsEvent<'_>) {
    event.commands.push(NativeCommand::new("car"));
    event.commands.push(NativeCommand::new("car remove"));
    event.commands.push(NativeCommand::new("car help"));
}

pub fn run(host: &mut dyn Host, event: &mut PlayerCommandEvent<'_>) -> Flow {
    let line = event.command.trim();
    let (head, rest) = match line.split_once(char::is_whitespace) {
        Some((head, rest)) => (head, rest.trim()),
        None => (line, ""),
    };
    if head != "car" {
        return Flow::Continue;
    }
    let Some(player) = player_of(host, event.player) else {
        event
            .replies
            .push(Text::literal("Не вижу вашего персонажа в мире."));
        return Flow::Cancel;
    };
    match rest {
        "" => place(&player, event),
        "remove" => remove(&player, event),
        "help" => help(event),
        other => event.replies.push(Text::literal(format!(
            "Неизвестный аргумент «{other}». /car, /car remove, /car help"
        ))),
    }
    Flow::Cancel
}

/// Where the command's player is: a LAN guest comes from the server's list,
/// the host (who has no UUID in `Host::player`) from the player in place.
struct Who {
    position: [f64; 3],
    dimension: String,
    yaw: f32,
}

fn player_of(host: &dyn Host, uuid: u128) -> Option<Who> {
    if let Some(guest) = host.server_players().into_iter().find(|p| p.uuid == uuid) {
        return Some(Who {
            position: guest.position,
            dimension: guest.dimension,
            yaw: guest.yaw,
        });
    }
    host.player().map(|p| Who {
        position: [p.x, p.y, p.z],
        dimension: p.dimension.to_owned(),
        yaw: p.yaw,
    })
}

fn place(player: &Who, event: &mut PlayerCommandEvent<'_>) {
    if state::car_count() >= MAX_CARS {
        event
            .replies
            .push(Text::literal(format!("Уже {MAX_CARS} машин — больше нельзя.")));
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
    event
        .replies
        .push(Text::literal("Машина поставлена. Правый клик — сесть."));
}

fn remove(player: &Who, event: &mut PlayerCommandEvent<'_>) {
    match state::nearest_car(player.position, &player.dimension, 12.0) {
        Some((id, _)) => {
            state::queue(Action::Remove { id });
            event.replies.push(Text::literal("Убираю машину."));
        }
        None => event
            .replies
            .push(Text::literal("Рядом нет машины (ищу в 12 блоках).")),
    }
}

fn help(event: &mut PlayerCommandEvent<'_>) {
    for line in [
        "/car — поставить машину перед собой",
        "/car remove — убрать ближайшую машину",
        "/car help — эта справка",
        "W газ, S тормоз/задний ход, A/D руль, Space ручник, Shift — выйти",
    ] {
        event.replies.push(Text::literal(line));
    }
}
