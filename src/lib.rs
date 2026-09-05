// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 Jonathan D.A. Jewell <j.d.a.jewell@open.ac.uk>

//! Exact, reversible three-dimensional dynamics for Pong-Ping.
//!
//! The host always calls [`Machine::step`] in ordinary wall-clock order. At a
//! paddle contact, however, the simulated program direction changes sign. The
//! contact interceptor applies a tensor-derived momentum shear at the same
//! instant. That shear is an involution when combined with the direction flip,
//! so [`Machine::undo`] can recover the previous state without a state-delta
//! log. It needs only the same external control value supplied to `step`.

#![forbid(unsafe_code)]

use std::error::Error;
use std::fmt;

/// A small exact three-vector used for position offsets, momentum, and torque.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Vec3 {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

impl Vec3 {
    #[must_use]
    pub const fn new(x: i64, y: i64, z: i64) -> Self {
        Self { x, y, z }
    }

    fn checked_add(self, rhs: Self) -> Result<Self, StepError> {
        Ok(Self {
            x: self.x.checked_add(rhs.x).ok_or(StepError::Overflow)?,
            y: self.y.checked_add(rhs.y).ok_or(StepError::Overflow)?,
            z: self.z.checked_add(rhs.z).ok_or(StepError::Overflow)?,
        })
    }

    fn checked_scale(self, scalar: i64) -> Result<Self, StepError> {
        Ok(Self {
            x: self.x.checked_mul(scalar).ok_or(StepError::Overflow)?,
            y: self.y.checked_mul(scalar).ok_or(StepError::Overflow)?,
            z: self.z.checked_mul(scalar).ok_or(StepError::Overflow)?,
        })
    }

    fn checked_cross(self, rhs: Self) -> Result<Self, StepError> {
        let x = self
            .y
            .checked_mul(rhs.z)
            .and_then(|a| self.z.checked_mul(rhs.y).and_then(|b| a.checked_sub(b)))
            .ok_or(StepError::Overflow)?;
        let y = self
            .z
            .checked_mul(rhs.x)
            .and_then(|a| self.x.checked_mul(rhs.z).and_then(|b| a.checked_sub(b)))
            .ok_or(StepError::Overflow)?;
        let z = self
            .x
            .checked_mul(rhs.y)
            .and_then(|a| self.y.checked_mul(rhs.x).and_then(|b| a.checked_sub(b)))
            .ok_or(StepError::Overflow)?;
        Ok(Self { x, y, z })
    }
}

/// A symmetric, integer-scaled second-moment tensor.
///
/// Its normal row and column are zero because the simulated program-direction
/// reversal supplies the normal bounce. The tangential block couples the Y and
/// Z contact offsets, which turns a flat retrace into a genuinely 3D path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MomentTensor3 {
    rows: [[i64; 3]; 3],
    scale: i64,
}

impl MomentTensor3 {
    /// The gameplay tensor has tangential block `[[2, 1], [1, 2]] / 2`.
    pub const STANDARD: Self = Self {
        rows: [[0, 0, 0], [0, 2, 1], [0, 1, 2]],
        scale: 2,
    };

    /// Construct a symmetric tensor with a strictly positive scale.
    pub fn new(rows: [[i64; 3]; 3], scale: i64) -> Result<Self, TensorError> {
        if scale <= 0 {
            return Err(TensorError::NonPositiveScale);
        }
        for (row_index, row) in rows.iter().enumerate() {
            for (column_index, coefficient) in row.iter().enumerate() {
                if *coefficient != rows[column_index][row_index] {
                    return Err(TensorError::NotSymmetric);
                }
            }
        }
        Ok(Self { rows, scale })
    }

    /// Apply the tensor using deterministic, symmetric integer rounding.
    pub fn apply(self, vector: Vec3) -> Result<Vec3, StepError> {
        let mut output = [0_i64; 3];
        let input = [vector.x, vector.y, vector.z];
        for (row_index, row) in self.rows.iter().enumerate() {
            let mut sum = 0_i64;
            for (coefficient, component) in row.iter().zip(input) {
                let term = coefficient
                    .checked_mul(component)
                    .ok_or(StepError::Overflow)?;
                sum = sum.checked_add(term).ok_or(StepError::Overflow)?;
            }
            output[row_index] = rounded_division(sum, self.scale);
        }
        Ok(Vec3::new(output[0], output[1], output[2]))
    }

    #[must_use]
    pub const fn rows(self) -> [[i64; 3]; 3] {
        self.rows
    }

    #[must_use]
    pub const fn scale(self) -> i64 {
        self.scale
    }
}

impl Default for MomentTensor3 {
    fn default() -> Self {
        Self::STANDARD
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TensorError {
    NonPositiveScale,
    NotSymmetric,
}

impl fmt::Display for TensorError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveScale => formatter.write_str("tensor scale must be positive"),
            Self::NotSymmetric => formatter.write_str("moment tensor must be symmetric"),
        }
    }
}

impl Error for TensorError {}

/// Which way the simulated program is executing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    Forward,
    Reverse,
}

impl Direction {
    #[must_use]
    pub const fn sign(self) -> i64 {
        match self {
            Self::Forward => 1,
            Self::Reverse => -1,
        }
    }

    #[must_use]
    pub const fn flipped(self) -> Self {
        match self {
            Self::Forward => Self::Reverse,
            Self::Reverse => Self::Forward,
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Forward => formatter.write_str("FORWARD >>"),
            Self::Reverse => formatter.write_str("<< REVERSE"),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Side {
    Left,
    Right,
}

impl fmt::Display for Side {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Left => formatter.write_str("left"),
            Self::Right => formatter.write_str("right"),
        }
    }
}

/// A reversible paddle translation expressed in program coordinates.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PaddleShift {
    pub y: i64,
    pub z: i64,
}

impl PaddleShift {
    #[must_use]
    pub const fn new(y: i64, z: i64) -> Self {
        Self { y, z }
    }
}

/// Exogenous input for one tick. Keep this value to undo that tick later.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Control {
    pub left: PaddleShift,
    pub right: PaddleShift,
}

impl Control {
    pub const NONE: Self = Self {
        left: PaddleShift::new(0, 0),
        right: PaddleShift::new(0, 0),
    };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Court {
    pub x_max: i64,
    pub y_max: i64,
    pub z_max: i64,
}

impl Court {
    pub const STANDARD: Self = Self {
        x_max: 30,
        y_max: 15,
        z_max: 15,
    };

    pub fn new(x_max: i64, y_max: i64, z_max: i64) -> Result<Self, CourtError> {
        if x_max < 2 || y_max < 2 || z_max < 2 {
            return Err(CourtError::TooSmall);
        }
        Ok(Self {
            x_max,
            y_max,
            z_max,
        })
    }
}

impl Default for Court {
    fn default() -> Self {
        Self::STANDARD
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CourtError {
    TooSmall,
}

impl fmt::Display for CourtError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("every court dimension must be at least two units")
    }
}

impl Error for CourtError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Paddle {
    centre_y: i64,
    centre_z: i64,
    half_y: i64,
    half_z: i64,
}

impl Paddle {
    #[must_use]
    pub const fn centre_y(self) -> i64 {
        self.centre_y
    }

    #[must_use]
    pub const fn centre_z(self) -> i64 {
        self.centre_z
    }

    #[must_use]
    pub const fn half_y(self) -> i64 {
        self.half_y
    }

    #[must_use]
    pub const fn half_z(self) -> i64 {
        self.half_z
    }

    fn contains(self, y: i64, z: i64) -> bool {
        (y - self.centre_y).abs() <= self.half_y && (z - self.centre_z).abs() <= self.half_z
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ball {
    x: i64,
    y_phase: i64,
    z_phase: i64,
    spin_moment: Vec3,
}

impl Ball {
    #[must_use]
    pub const fn spin_moment(self) -> Vec3 {
        self.spin_moment
    }
}

/// Interpreted reversible code controlling one transit step.
///
/// Keeping this separate from `Ball` is deliberate: the executor direction
/// selects the instruction or its inverse, while the meta-interceptor rewrites
/// the instruction at contact. The altered backward program therefore follows
/// a new path instead of replaying the path that arrived at the paddle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransitProgram {
    translation: Vec3,
    patch_phase: i64,
}

impl TransitProgram {
    #[must_use]
    pub const fn translation(self) -> Vec3 {
        self.translation
    }

    #[must_use]
    pub const fn patch_phase(self) -> i64 {
        self.patch_phase
    }

    fn apply_patch(&mut self, patch: Vec3, direction: Direction) -> Result<(), StepError> {
        let signed_patch = patch.checked_scale(direction.sign())?;
        self.translation = self.translation.checked_add(signed_patch)?;
        self.patch_phase = self
            .patch_phase
            .checked_add(direction.sign())
            .ok_or(StepError::Overflow)?;
        Ok(())
    }
}

/// An observable event emitted by an accepted wall-clock tick.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepEvent {
    Drift,
    Intercept {
        side: Side,
        direction_before: Direction,
        lever_arm: Vec3,
        tensor_impulse: Vec3,
        torque_moment: Vec3,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepError {
    Miss {
        side: Side,
        ball_y: i64,
        ball_z: i64,
    },
    PaddleOutOfBounds {
        side: Side,
    },
    NoTickToUndo,
    InconsistentState,
    Overflow,
}

impl fmt::Display for StepError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Miss {
                side,
                ball_y,
                ball_z,
            } => write!(formatter, "{side} paddle missed at y={ball_y}, z={ball_z}"),
            Self::PaddleOutOfBounds { side } => {
                write!(formatter, "{side} paddle control would leave the court")
            }
            Self::NoTickToUndo => formatter.write_str("there is no accepted tick to undo"),
            Self::InconsistentState => formatter.write_str("machine state violates court bounds"),
            Self::Overflow => formatter.write_str("exact integer state exceeded i64 capacity"),
        }
    }
}

impl Error for StepError {}

/// The complete reversible simulation state.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Machine {
    court: Court,
    ball: Ball,
    transit_program: TransitProgram,
    left: Paddle,
    right: Paddle,
    direction: Direction,
    logical_time: i64,
    wall_ticks: u64,
    contact_tensor: MomentTensor3,
}

impl Machine {
    /// A stable rally whose tensor impulse turns Y motion into Z motion and back.
    #[must_use]
    pub fn standard() -> Self {
        let court = Court::STANDARD;
        let paddle = Paddle {
            centre_y: 8,
            centre_z: 8,
            half_y: 4,
            half_z: 3,
        };
        Self {
            court,
            ball: Ball {
                x: 15,
                y_phase: 8,
                z_phase: 8,
                spin_moment: Vec3::new(0, 0, 0),
            },
            transit_program: TransitProgram {
                translation: Vec3::new(1, 1, 0),
                patch_phase: 0,
            },
            left: paddle,
            right: paddle,
            direction: Direction::Forward,
            logical_time: 0,
            wall_ticks: 0,
            contact_tensor: MomentTensor3::STANDARD,
        }
    }

    #[must_use]
    pub const fn court(&self) -> Court {
        self.court
    }

    #[must_use]
    pub const fn ball(&self) -> Ball {
        self.ball
    }

    #[must_use]
    pub const fn transit_program(&self) -> TransitProgram {
        self.transit_program
    }

    #[must_use]
    pub const fn left_paddle(&self) -> Paddle {
        self.left
    }

    #[must_use]
    pub const fn right_paddle(&self) -> Paddle {
        self.right
    }

    #[must_use]
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    #[must_use]
    pub const fn logical_time(&self) -> i64 {
        self.logical_time
    }

    #[must_use]
    pub const fn wall_ticks(&self) -> u64 {
        self.wall_ticks
    }

    #[must_use]
    pub const fn contact_tensor(&self) -> MomentTensor3 {
        self.contact_tensor
    }

    #[must_use]
    pub fn ball_position(&self) -> Vec3 {
        Vec3::new(
            self.ball.x,
            fold_phase(self.ball.y_phase, self.court.y_max),
            fold_phase(self.ball.z_phase, self.court.z_max),
        )
    }

    /// Advance one host tick. On error, the state is left byte-for-byte unchanged.
    pub fn step(&mut self, control: Control) -> Result<StepEvent, StepError> {
        let mut candidate = self.clone();
        let event = candidate.step_inner(control)?;
        *self = candidate;
        Ok(event)
    }

    /// Invert one accepted host tick using the same external control value.
    ///
    /// No previous simulation state is consulted. A UI may keep a control tape
    /// because user input originates outside the reversible machine.
    pub fn undo(&mut self, control: Control) -> Result<StepEvent, StepError> {
        let mut candidate = self.clone();
        let event = candidate.undo_inner(control)?;
        *self = candidate;
        Ok(event)
    }

    fn step_inner(&mut self, control: Control) -> Result<StepEvent, StepError> {
        let approach_direction = self.direction;
        self.apply_control(control, approach_direction, 1)?;

        let event = match self.outward_contact() {
            Some(side) => {
                let paddle = self.paddle(side);
                let position = self.ball_position();
                if !paddle.contains(position.y, position.z) {
                    return Err(StepError::Miss {
                        side,
                        ball_y: position.y,
                        ball_z: position.z,
                    });
                }
                self.intercept(side, control_for_side(control, side))?
            }
            None => StepEvent::Drift,
        };

        self.drift(1)?;
        self.logical_time = self
            .logical_time
            .checked_add(self.direction.sign())
            .ok_or(StepError::Overflow)?;
        self.wall_ticks = self.wall_ticks.checked_add(1).ok_or(StepError::Overflow)?;
        Ok(event)
    }

    fn undo_inner(&mut self, control: Control) -> Result<StepEvent, StepError> {
        if self.wall_ticks == 0 {
            return Err(StepError::NoTickToUndo);
        }

        self.logical_time = self
            .logical_time
            .checked_sub(self.direction.sign())
            .ok_or(StepError::Overflow)?;
        self.drift(-1)?;

        let event = match self.inward_contact() {
            Some(side) => self.intercept(side, control_for_side(control, side))?,
            None => StepEvent::Drift,
        };

        self.apply_control(control, self.direction, -1)?;
        self.wall_ticks -= 1;
        Ok(event)
    }

    fn apply_control(
        &mut self,
        control: Control,
        direction: Direction,
        operation: i64,
    ) -> Result<(), StepError> {
        let factor = direction
            .sign()
            .checked_mul(operation)
            .ok_or(StepError::Overflow)?;
        move_paddle(&mut self.left, control.left, factor, self.court, Side::Left)?;
        move_paddle(
            &mut self.right,
            control.right,
            factor,
            self.court,
            Side::Right,
        )?;
        Ok(())
    }

    fn drift(&mut self, operation: i64) -> Result<(), StepError> {
        let factor = self
            .direction
            .sign()
            .checked_mul(operation)
            .ok_or(StepError::Overflow)?;
        // This is the reversible executor: forward selects the instruction;
        // reverse selects its exact additive inverse.
        let displacement = self.transit_program.translation.checked_scale(factor)?;
        self.ball.x = self
            .ball
            .x
            .checked_add(displacement.x)
            .ok_or(StepError::Overflow)?;
        if !(0..=self.court.x_max).contains(&self.ball.x) {
            return Err(StepError::InconsistentState);
        }
        self.ball.y_phase = advance_phase(self.ball.y_phase, displacement.y, self.court.y_max)?;
        self.ball.z_phase = advance_phase(self.ball.z_phase, displacement.z, self.court.z_max)?;
        Ok(())
    }

    fn intercept(&mut self, side: Side, paddle_shift: PaddleShift) -> Result<StepEvent, StepError> {
        let direction_before = self.direction;
        let position = self.ball_position();
        let paddle = self.paddle(side);
        let lever_arm = Vec3::new(
            0,
            position.y - paddle.centre_y,
            position.z - paddle.centre_z,
        );
        let tensor_impulse = self
            .contact_tensor
            .apply(lever_arm)?
            .checked_add(Vec3::new(0, paddle_shift.y, paddle_shift.z))?;

        // A unit normal momentum reverses through the direction flip, so the
        // corresponding physical normal impulse has magnitude two.
        let physical_impulse = Vec3::new(2, tensor_impulse.y, tensor_impulse.z);
        let torque_moment = lever_arm.checked_cross(physical_impulse)?;
        let signed_torque = torque_moment.checked_scale(direction_before.sign())?;
        self.transit_program.apply_patch(
            Vec3::new(0, tensor_impulse.y, tensor_impulse.z),
            direction_before,
        )?;
        self.ball.spin_moment = self.ball.spin_moment.checked_add(signed_torque)?;
        self.direction = self.direction.flipped();

        Ok(StepEvent::Intercept {
            side,
            direction_before,
            lever_arm,
            tensor_impulse,
            torque_moment,
        })
    }

    fn outward_contact(&self) -> Option<Side> {
        match (self.ball.x, self.direction) {
            (0, Direction::Reverse) => Some(Side::Left),
            (x, Direction::Forward) if x == self.court.x_max => Some(Side::Right),
            _ => None,
        }
    }

    fn inward_contact(&self) -> Option<Side> {
        match (self.ball.x, self.direction) {
            (0, Direction::Forward) => Some(Side::Left),
            (x, Direction::Reverse) if x == self.court.x_max => Some(Side::Right),
            _ => None,
        }
    }

    fn paddle(&self, side: Side) -> Paddle {
        match side {
            Side::Left => self.left,
            Side::Right => self.right,
        }
    }
}

impl Default for Machine {
    fn default() -> Self {
        Self::standard()
    }
}

fn control_for_side(control: Control, side: Side) -> PaddleShift {
    match side {
        Side::Left => control.left,
        Side::Right => control.right,
    }
}

fn move_paddle(
    paddle: &mut Paddle,
    shift: PaddleShift,
    factor: i64,
    court: Court,
    side: Side,
) -> Result<(), StepError> {
    let y = shift
        .y
        .checked_mul(factor)
        .and_then(|delta| paddle.centre_y.checked_add(delta))
        .ok_or(StepError::Overflow)?;
    let z = shift
        .z
        .checked_mul(factor)
        .and_then(|delta| paddle.centre_z.checked_add(delta))
        .ok_or(StepError::Overflow)?;
    if y - paddle.half_y < 0
        || y + paddle.half_y > court.y_max
        || z - paddle.half_z < 0
        || z + paddle.half_z > court.z_max
    {
        return Err(StepError::PaddleOutOfBounds { side });
    }
    paddle.centre_y = y;
    paddle.centre_z = z;
    Ok(())
}

fn advance_phase(phase: i64, displacement: i64, maximum: i64) -> Result<i64, StepError> {
    let period = maximum.checked_mul(2).ok_or(StepError::Overflow)?;
    phase
        .checked_add(displacement)
        .map(|next| next.rem_euclid(period))
        .ok_or(StepError::Overflow)
}

fn fold_phase(phase: i64, maximum: i64) -> i64 {
    let period = maximum * 2;
    let wrapped = phase.rem_euclid(period);
    if wrapped <= maximum {
        wrapped
    } else {
        period - wrapped
    }
}

fn rounded_division(value: i64, divisor: i64) -> i64 {
    if value >= 0 {
        (value + divisor / 2) / divisor
    } else {
        -((-value + divisor / 2) / divisor)
    }
}

/// Render the 3D state as synchronized front (X-Y) and top (X-Z) projections.
#[must_use]
pub fn render(machine: &Machine) -> String {
    let court = machine.court();
    let position = machine.ball_position();
    let width = usize::try_from(court.x_max + 1).expect("standard court width fits usize");
    let front_height = usize::try_from(court.y_max + 1).expect("court height fits usize");
    let top_height = usize::try_from(court.z_max + 1).expect("court depth fits usize");
    let mut front = vec![vec![' '; width]; front_height];
    let mut top = vec![vec![' '; width]; top_height];

    draw_projection(
        &mut front,
        machine.left_paddle().centre_y(),
        machine.left_paddle().half_y(),
        machine.right_paddle().centre_y(),
        machine.right_paddle().half_y(),
        position.x,
        position.y,
    );
    draw_projection(
        &mut top,
        machine.left_paddle().centre_z(),
        machine.left_paddle().half_z(),
        machine.right_paddle().centre_z(),
        machine.right_paddle().half_z(),
        position.x,
        position.z,
    );

    let instruction = machine.transit_program().translation();
    let spin = machine.ball().spin_moment();
    let mut output = format!(
        "PONG-PING 3D  | wall tick {:>5} | program time {:>4} | {}\n\
         ball=({:>2},{:>2},{:>2})  transit opcode=({:>2},{:>2},{:>2})  spin moment=({:>3},{:>3},{:>3})\n\
         FRONT: X-Y{:width_gap$}TOP: X-Z\n",
        machine.wall_ticks(),
        machine.logical_time(),
        machine.direction(),
        position.x,
        position.y,
        position.z,
        instruction.x,
        instruction.y,
        instruction.z,
        spin.x,
        spin.y,
        spin.z,
        "",
        width_gap = width.saturating_sub(10) + 5,
    );
    let rows = front_height.max(top_height);
    for row in 0..rows {
        if let Some(line) = front.get(row) {
            output.extend(line);
        } else {
            output.push_str(&" ".repeat(width));
        }
        output.push_str("     ");
        if let Some(line) = top.get(row) {
            output.extend(line);
        }
        output.push('\n');
    }
    output
}

fn draw_projection(
    grid: &mut [Vec<char>],
    left_centre: i64,
    left_half: i64,
    right_centre: i64,
    right_half: i64,
    ball_x: i64,
    ball_axis: i64,
) {
    let width = grid.first().map_or(0, Vec::len);
    if width == 0 {
        return;
    }
    for (axis, row) in grid.iter_mut().enumerate() {
        row[0] = if (i64::try_from(axis).expect("axis fits i64") - left_centre).abs() <= left_half {
            '#'
        } else {
            '|'
        };
        row[width - 1] =
            if (i64::try_from(axis).expect("axis fits i64") - right_centre).abs() <= right_half {
                '#'
            } else {
                '|'
            };
    }
    if let Some(row) = grid.get_mut(usize::try_from(ball_axis).expect("ball axis fits usize")) {
        row[usize::try_from(ball_x).expect("ball x fits usize")] = 'O';
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interceptor_is_an_involution() {
        let mut machine = Machine::standard();
        for _ in 0..15 {
            machine.step(Control::NONE).unwrap();
        }
        let contact = machine.clone();
        machine
            .intercept(Side::Right, PaddleShift::default())
            .unwrap();
        machine
            .intercept(Side::Right, PaddleShift::default())
            .unwrap();
        assert_eq!(machine, contact);
    }

    #[test]
    fn lossy_angle_setter_is_detected_by_positive_control() {
        fn naive_setter(program: &mut TransitProgram) {
            program.translation.y = 2;
            program.translation.z = -1;
        }

        let mut first = Machine::standard().transit_program;
        let mut second = first;
        second.translation.y = 99;
        second.translation.z = 42;
        assert_ne!(first, second);
        naive_setter(&mut first);
        naive_setter(&mut second);
        assert_eq!(
            first, second,
            "the naive setter destroys incoming information"
        );
    }
}
