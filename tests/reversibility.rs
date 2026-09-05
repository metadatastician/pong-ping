// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 Jonathan D.A. Jewell <j.d.a.jewell@open.ac.uk>

use pong_ping::{render, Control, Direction, Machine, MomentTensor3, PaddleShift, StepError, Vec3};

#[test]
fn thousands_of_steps_round_trip_exactly() {
    let initial = Machine::standard();
    let mut machine = initial.clone();
    let controls = vec![Control::NONE; 4_096];

    for control in controls.iter().copied() {
        machine.step(control).unwrap();
    }
    assert!(machine.wall_ticks() > 0);

    for control in controls.into_iter().rev() {
        machine.undo(control).unwrap();
    }
    assert_eq!(machine, initial);
}

#[test]
fn paddle_contact_changes_program_direction_and_third_axis() {
    let mut machine = Machine::standard();
    for _ in 0..15 {
        machine.step(Control::NONE).unwrap();
    }
    let before = machine.ball_position();
    assert_eq!(machine.direction(), Direction::Forward);

    machine.step(Control::NONE).unwrap();

    assert_eq!(machine.direction(), Direction::Reverse);
    assert_ne!(machine.ball_position().z, before.z);
    assert_ne!(machine.ball().spin_moment(), Vec3::default());
}

#[test]
fn tensor_couples_y_contact_into_z_impulse() {
    let impulse = MomentTensor3::STANDARD.apply(Vec3::new(0, -2, 0)).unwrap();
    assert_ne!(impulse.z, 0);
}

#[test]
fn controlled_code_rewrite_round_trips() {
    let mut machine = Machine::standard();
    for _ in 0..15 {
        machine.step(Control::NONE).unwrap();
    }
    let contact_state = machine.clone();
    let control = Control {
        left: PaddleShift::default(),
        right: PaddleShift::new(1, 0),
    };

    machine.step(control).unwrap();
    assert_ne!(machine.transit_program(), contact_state.transit_program());
    machine.undo(control).unwrap();

    assert_eq!(machine, contact_state);
}

#[test]
fn rejected_control_is_transactional() {
    let mut machine = Machine::standard();
    let initial = machine.clone();
    let invalid = Control {
        left: PaddleShift::default(),
        right: PaddleShift::new(4, 0),
    };

    assert_eq!(
        machine.step(invalid),
        Err(StepError::PaddleOutOfBounds {
            side: pong_ping::Side::Right
        })
    );
    assert_eq!(machine, initial);
}

#[test]
fn renderer_exposes_both_orthographic_views() {
    let picture = render(&Machine::standard());
    assert!(picture.contains("FRONT: X-Y"));
    assert!(picture.contains("TOP: X-Z"));
    assert!(picture.contains('O'));
}
