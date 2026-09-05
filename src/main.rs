// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 Jonathan D.A. Jewell <j.d.a.jewell@open.ac.uk>

#![forbid(unsafe_code)]

use pong_ping::{render, Control, Direction, Machine, PaddleShift, StepEvent};
use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::Duration;

fn main() -> ExitCode {
    let arguments: Vec<String> = env::args().skip(1).collect();
    let command = arguments.first().map_or("play", String::as_str);
    let ticks = arguments
        .get(1)
        .and_then(|value| value.parse::<usize>().ok());

    let result = match command {
        "play" => play(),
        "demo" => demo(ticks.unwrap_or(120), 55),
        "trace" => trace(ticks.unwrap_or(128)),
        "prove" | "proof" => prove(ticks.unwrap_or(4_096)),
        "help" | "--help" | "-h" => {
            help();
            Ok(())
        }
        unknown => Err(format!("unknown command {unknown:?}; run `pong-ping help`")),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("pong-ping: {message}");
            ExitCode::FAILURE
        }
    }
}

fn help() {
    println!(
        "Pong-Ping — reversible 3D tensor-moment Pong\n\
         \n\
         USAGE\n\
           pong-ping play             turn-based playable mode (default)\n\
           pong-ping demo [ticks]     animated stable rally\n\
           pong-ping trace [ticks]    print tensor interceptions, then rewind\n\
           pong-ping prove [ticks]    exact round-trip verification\n"
    );
}

fn prove(ticks: usize) -> Result<(), String> {
    let initial = Machine::standard();
    let mut machine = initial.clone();
    let tape = vec![Control::NONE; ticks];

    for (index, control) in tape.iter().copied().enumerate() {
        machine
            .step(control)
            .map_err(|error| format!("step {index} failed: {error}"))?;
    }
    let apex = machine.clone();
    for (index, control) in tape.iter().copied().rev().enumerate() {
        machine
            .undo(control)
            .map_err(|error| format!("undo {index} failed: {error}"))?;
    }

    if machine != initial {
        return Err("round trip did not restore the exact initial state".to_owned());
    }
    println!("PASS: {ticks} forward host ticks + {ticks} inverse ticks restored the exact state.");
    println!(
        "Apex: program direction={}, logical time={}, ball={:?}, transit opcode={:?}, spin={:?}",
        apex.direction(),
        apex.logical_time(),
        apex.ball_position(),
        apex.transit_program().translation(),
        apex.ball().spin_moment(),
    );
    Ok(())
}

fn trace(ticks: usize) -> Result<(), String> {
    let initial = Machine::standard();
    let mut machine = initial.clone();
    let mut tape = Vec::with_capacity(ticks);
    let mut contacts = 0_usize;

    for _ in 0..ticks {
        let control = Control::NONE;
        let event = machine.step(control).map_err(|error| error.to_string())?;
        tape.push(control);
        if let StepEvent::Intercept {
            side,
            direction_before,
            lever_arm,
            tensor_impulse,
            torque_moment,
        } = event
        {
            contacts += 1;
            println!(
                "tick {:>4}: {side} intercept | {direction_before} -> {} | lever={lever_arm:?} | tensor impulse={tensor_impulse:?} | torque={torque_moment:?}",
                machine.wall_ticks(),
                machine.direction(),
            );
        }
    }
    for control in tape.into_iter().rev() {
        machine.undo(control).map_err(|error| error.to_string())?;
    }
    if machine != initial {
        return Err("trace rewind failed to restore the initial state".to_owned());
    }
    println!("PASS: observed {contacts} tensor contacts and exactly rewound all {ticks} ticks.");
    Ok(())
}

fn demo(ticks: usize, delay_ms: u64) -> Result<(), String> {
    let initial = Machine::standard();
    let mut machine = initial.clone();
    let mut tape = Vec::with_capacity(ticks);
    for _ in 0..ticks {
        print!("\x1b[2J\x1b[H{}", render(&machine));
        println!("Stable demonstration rally; Ctrl-C exits.");
        io::stdout().flush().map_err(|error| error.to_string())?;
        let control = Control::NONE;
        machine.step(control).map_err(|error| error.to_string())?;
        tape.push(control);
        thread::sleep(Duration::from_millis(delay_ms));
    }
    for control in tape.into_iter().rev() {
        machine.undo(control).map_err(|error| error.to_string())?;
    }
    if machine != initial {
        return Err("demonstration rewind failed".to_owned());
    }
    println!("\nPASS: demonstration rewound to its exact initial state.");
    Ok(())
}

fn play() -> Result<(), String> {
    let mut machine = Machine::standard();
    let mut tape = Vec::new();
    println!("Commands: Enter=step, u=undo, q=quit");
    println!("Left paddle: w/s (Y), a/d (Z). Right: i/k (Y), j/l (Z).\n");

    loop {
        println!("{}", render(&machine));
        print!("pong-ping> ");
        io::stdout().flush().map_err(|error| error.to_string())?;
        let mut line = String::new();
        if io::stdin()
            .read_line(&mut line)
            .map_err(|error| error.to_string())?
            == 0
        {
            return Ok(());
        }
        let command = line.trim().to_ascii_lowercase();
        if command == "q" || command == "quit" {
            return Ok(());
        }
        if command == "u" || command == "undo" {
            match tape.pop() {
                Some(control) => {
                    machine.undo(control).map_err(|error| error.to_string())?;
                }
                None => println!("Nothing to undo."),
            }
            continue;
        }

        let control = physical_control(&machine, &command)?;
        match machine.step(control) {
            Ok(StepEvent::Intercept { side, .. }) => {
                println!("Tensor interception at the {side} paddle; code direction flipped.");
                tape.push(control);
            }
            Ok(StepEvent::Drift) => tape.push(control),
            Err(error) => println!("Tick rejected without changing state: {error}"),
        }
    }
}

fn physical_control(machine: &Machine, command: &str) -> Result<Control, String> {
    let mut left_y = 0_i64;
    let mut left_z = 0_i64;
    let mut right_y = 0_i64;
    let mut right_z = 0_i64;
    for character in command.chars() {
        match character {
            'w' => left_y -= 1,
            's' => left_y += 1,
            'a' => left_z -= 1,
            'd' => left_z += 1,
            'i' => right_y -= 1,
            'k' => right_y += 1,
            'j' => right_z -= 1,
            'l' => right_z += 1,
            '.' | ' ' => {}
            other => return Err(format!("unknown play key {other:?}")),
        }
    }
    // Controls are stored in program coordinates. Multiplying a desired
    // physical shift by the current sign keeps the UI intuitive in either
    // simulated execution direction.
    let sign = match machine.direction() {
        Direction::Forward => 1,
        Direction::Reverse => -1,
    };
    Ok(Control {
        left: PaddleShift::new(left_y * sign, left_z * sign),
        right: PaddleShift::new(right_y * sign, right_z * sign),
    })
}
