use std::collections::VecDeque;

use oort_api::prelude::*;

use super::math::kinematics::{Heading, Position, Velocity};

////////////////////////////////////////////////////////////////

#[derive(Clone, Copy)]
pub enum Colour {
    White = 0xFFFFFF,

    Red = 0xFF0000,
    Green = 0x00FF00,
    Blue = 0x0000FF,

    Yellow = 0xFFFF00,
    Teal = 0x00FFFF,
    Purple = 0xFF00FF,
}

////////////////////////////////////////////////////////////////

pub fn contains_nan(vec: &Vec2) -> bool {
    return vec.x.is_nan() || vec.y.is_nan();
}

////////////////////////////////////////////////////////////////

pub fn line<T: Position, U: Position>(start: &T, end: &U, colour: Colour) {
    if !contains_nan(&start.position()) && !contains_nan(&end.position()) {
        draw_line(start.position(), end.position(), colour as u32);
    }
}
pub fn circle<T: Position>(position: &T, radius: f64, colour: Colour) {
    if !contains_nan(&position.position()) && !radius.is_nan() {
        draw_polygon(position.position(), radius, 8, 0.0, colour as u32);
    }
}

pub fn triangle<T: Position>(position: &T, radius: f64, colour: Colour) {
    if !contains_nan(&position.position()) && !radius.is_nan() {
        draw_triangle(position.position(), radius, colour as u32);
    }
}

pub fn square<T: Position>(position: &T, radius: f64, colour: Colour) {
    if !contains_nan(&position.position()) && !radius.is_nan() {
        draw_square(position.position(), radius, colour as u32);
    }
}

pub fn diamond<T: Position>(position: &T, radius: f64, colour: Colour) {
    if !contains_nan(&position.position()) && !radius.is_nan() {
        draw_diamond(position.position(), radius, colour as u32);
    }
}

pub fn regular_polygon<T: Position>(
    centre: &T,
    radius: f64,
    sides: u32,
    angle: f64,
    colour: Colour,
) {
    draw_polygon(
        centre.position(),
        radius,
        sides as i32,
        angle,
        colour as u32,
    );
}

////////////////////////////////////////////////////////////////

pub fn heading<T: Position + Heading>(vessel: &T) {
    line(
        vessel,
        &(vec2(1.0, 0.0).rotate(vessel.heading()) * 100000.0),
        Colour::Green,
    );
}

pub fn course<T: Velocity>(vessel: &T) {
    line(vessel, &(vessel.velocity() * 100000.0), Colour::Blue);
}

pub fn aim_reticle<T: Position>(position: &T) {
    triangle(position, 10.0, Colour::Red);
}

pub struct Trail {
    points: VecDeque<Vec2>,
}

impl Trail {
    pub fn with_length(length: usize) -> Self {
        return Self {
            points: VecDeque::<Vec2>::with_capacity(length),
        };
    }

    pub fn update<T: Position>(&mut self, point: &T) {
        if self.points.len() == self.points.capacity() {
            self.points.pop_front();
        }
        if !point.position().x.is_nan() && !point.position().y.is_nan() {
            self.points.push_back(point.position());
        }
    }

    pub fn draw(&self, colour: Colour) {
        for (p1, p2) in std::iter::zip(self.points.iter(), self.points.iter().skip(1)) {
            line(p1, p2, colour);
        }
    }
}

pub struct SimShot {
    position: Vec2,
    heading: f64,
    speed: f64,
    colour: Colour,

    pub time_alive: f64,
}

impl SimShot {
    pub fn new(position: Vec2, heading: f64, colour: Colour) -> Self {
        return Self {
            position,
            heading,
            speed: 1000.0,
            colour,
            time_alive: 0.0,
        };
    }

    pub fn tick(&mut self) {
        self.position += vec2(self.speed, 0.0).rotate(self.heading) * TICK_LENGTH;
        self.time_alive += TICK_LENGTH;
        diamond(&self.position, 3.0, self.colour);
    }
}

pub struct SimGun {
    cooldown: f64,
    last_shot_time: f64,

    shots: VecDeque<SimShot>,
}

impl SimGun {
    pub fn new(cooldown: f64) -> Self {
        return Self {
            cooldown,
            last_shot_time: current_time(),
            shots: VecDeque::default(),
        };
    }

    pub fn shoot(&mut self, position: Vec2, heading: f64, colour: Colour) {
        if (self.last_shot_time + self.cooldown) < current_time() {
            self.shots
                .push_back(SimShot::new(position, heading, colour));
            self.last_shot_time = current_time();
        }
    }

    pub fn tick(&mut self) {
        if let Some(shot) = self.shots.front() {
            if shot.time_alive > 3.0 {
                self.shots.pop_front();
            }
        }

        for shot in &mut self.shots {
            shot.tick();
        }
    }
}
