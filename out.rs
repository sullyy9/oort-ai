pub mod oort_ai {
#![allow(clippy::needless_return)]
use oort_api::prelude::*;
pub mod control {
use oort_api::prelude::*;
use super::math::kinematics::{Acceleration, AngularVelocity, Position, Velocity};
pub trait Translation: Position {
    fn set_acceleration(&mut self, acceleration: Vec2);
    fn accelerate_towards<T: Position>(&self, target: &T) {
        let target_vector = target.position() - self.position();
        accelerate(target_vector);
    }
}
pub trait Rotation: AngularVelocity + Velocity + Sized {
    fn set_angular_acceleration(&mut self, angular_acceleration: f64);
    fn turn_to_face<T: Position>(&mut self, target: &T) {
        let target_bearing = self.relative_bearing_to(target);
        let velocity = 2.0 * target_bearing.abs().sqrt();
        let velocity = if target_bearing < 0.0 {
            -velocity
        } else {
            velocity
        };
        let acceleration = (velocity - self.angular_velocity()) / TICK_LENGTH;
        self.set_angular_acceleration(acceleration);
    }
    fn turn_to_track<T: Acceleration>(&mut self, target: &T) {
        let target_bearing = self.relative_bearing_to(target);
        let decel = if target_bearing < 0.0 { 2.0 } else { -2.0 };
        let max_velocity = f64::sqrt(
            (target.orbital_velocity_to(self).powf(2.0) - (2.0 * decel * target_bearing)).abs(),
        );
        let max_velocity = if target_bearing.is_sign_negative() {
            -max_velocity
        } else {
            max_velocity
        };
        let acceleration = (max_velocity - self.angular_velocity()) / TICK_LENGTH;
        self.set_angular_acceleration(acceleration);
        debug!("----------------");
        debug!("target bearing:   {}", target_bearing);
        debug!("target velocity:  {}", max_velocity);
        debug!("orb acceleration: {}", target.orbital_acceleration_to(self));
        debug!("current velocity: {}", self.angular_velocity());
        debug!("acceleration:     {}", acceleration);
        debug!("----------------");
    }
}
}
pub mod draw {
use std::collections::VecDeque;
use oort_api::prelude::*;
use super::math::kinematics::{Heading, Position, Velocity};
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
pub fn contains_nan(vec: &Vec2) -> bool {
    return vec.x.is_nan() || vec.y.is_nan();
}
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
}
pub mod math {
pub mod firing_solution {
use oort_api::prelude::*;
use super::{
    kinematics::{Acceleration, Position, Velocity},
    polynomial::{self, Roots},
};
pub struct FiringSolution {
    impact_time: f64,
    impact_point: Vec2,
    target_velocity: Vec2,
    target_acceleration: Vec2,
}
impl Position for FiringSolution {
    fn position(&self) -> Vec2 {
        return self.impact_point;
    }
}
impl Velocity for FiringSolution {
    fn velocity(&self) -> Vec2 {
        return self.target_velocity;
    }
}
impl Acceleration for FiringSolution {
    fn acceleration(&self) -> Vec2 {
        return self.target_acceleration;
    }
}
impl FiringSolution {
    pub fn new<T: Velocity, U: Acceleration>(
        shooter: &T,
        projectile_speed: f64,
        target: &U,
    ) -> Option<Self> {
        let tarpos = target.position_relative_to(shooter);
        let tarvel = target.velocity_relative_to(shooter);
        let taracc = target.acceleration();
        let a = taracc.dot(taracc) / 4.0;
        let b = taracc.dot(tarvel);
        let c = tarvel.dot(tarvel) + taracc.dot(tarpos) - projectile_speed.powf(2.0);
        let d = 2.0 * tarvel.dot(tarpos);
        let e = tarpos.dot(tarpos);
        let roots = polynomial::find_roots_quartic(a, b, c, d, e);
        let impact_time = if let Roots::Four(roots) = roots {
            debug!("Root0: {}", roots[0]);
            debug!("Root1: {}", roots[1]);
            debug!("Root2: {}", roots[2]);
            debug!("Root3: {}", roots[3]);
            roots[2]
        } else if let Roots::Two(roots) = roots {
            debug!("Root0: {}", roots[0]);
            debug!("Root1: {}", roots[1]);
            roots[0]
        } else {
            f64::NAN
        };
        if impact_time.is_nan() {
            return None;
        }
        let impact_point =
            target.position() + ((tarvel * impact_time) + (0.5 * taracc * impact_time.powf(2.0)));
        return Some(Self {
            impact_time,
            impact_point,
            target_velocity: target.velocity(),
            target_acceleration: target.acceleration(),
        });
    }
}
}
pub mod geometry {
#![allow(dead_code)]
pub mod circle {
use oort_api::prelude::*;
use super::{
    draw::{self, Colour},
    kinematics::Position,
};
#[derive(Clone, PartialEq, Debug)]
pub struct Circle {
    pub centre: Vec2,
    pub radius: f64,
}
impl Circle {
    pub fn new<T: Position>(centre: &T, radius: f64) -> Self {
        return Self {
            centre: centre.position(),
            radius,
        };
    }
}
impl Circle {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        return point.position().distance_to(&self.centre) < self.radius;
    }
}
impl Circle {
    pub fn draw(&self, colour: Colour) {
        draw::regular_polygon(&self.centre, self.radius, 8, 0.0, colour)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_contains() {
        let circle = Circle::new(&vec2(0.0, 0.0), 5.0);
        assert!(circle.contains(&vec2(2.0, 2.0)));
        assert!(circle.contains(&vec2(-2.0, -2.0)));
        assert!(!circle.contains(&vec2(6.0, 0.0)));
        assert!(!circle.contains(&vec2(-6.0, 0.0)));
        assert!(!circle.contains(&vec2(-6.0, 5.0)));
        assert!(!circle.contains(&vec2(5.0, 0.0)));
        assert!(!circle.contains(&vec2(0.0, 5.0)));
    }
}
}
pub mod ellipse {
use oort_api::prelude::*;
use super::{draw, kinematics::Position};
pub struct Ellipse {
    centre: Vec2,
    orientation: f64,
    width: f64,
    height: f64,
}
impl Position for Ellipse {
    fn position(&self) -> Vec2 {
        return self.centre;
    }
}
impl Ellipse {
    pub fn new<T: Position>(centre: &T, orientation: f64, width: f64, height: f64) -> Self {
        return if width > height {
            Self {
                centre: centre.position(),
                orientation: orientation + std::f64::consts::FRAC_PI_2,
                width: height,
                height: width,
            }
        } else {
            return Self {
                centre: centre.position(),
                orientation,
                width,
                height,
            };
        };
    }
}
impl Ellipse {
    pub fn translate(&mut self, vector: &Vec2) {
        self.centre += vector;
    }
    pub fn expand(&mut self, value: f64) {
        self.width += value * 2.0;
        self.height += value * 2.0;
    }
}
impl Ellipse {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        let point = point.position_relative_to(self);
        let point = point.rotate(-self.orientation);
        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let xpart = (point.x).powi(2) / a.powi(2);
        let ypart = (point.y).powi(2) / b.powi(2);
        return (xpart + ypart) <= 1.0;
    }
    pub fn max_distance_to<T: Position>(&self, point: &T) -> f64 {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;
        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );
        println!("angle: {}", angle);
        println!("radius: {}", radius);
        debug!("angle: {}", angle);
        debug!("radius: {}", radius);
        return centre_distance + radius;
    }
    pub fn min_distance_to<T: Position>(&self, point: &T) -> f64 {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;
        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );
        return (centre_distance - radius).abs();
    }
    pub fn minmax_distance_to<T: Position>(&self, point: &T) -> (f64, f64) {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;
        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );
        return ((centre_distance - radius).abs(), centre_distance + radius);
    }
}
impl Ellipse {
    pub fn draw(&self, colour: draw::Colour) {
        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let mut points = Vec::new();
        const POINTS: usize = 40;
        let step = self.height / (POINTS / 2) as f64;
        let y = |x: f64| f64::sqrt(b.powf(2.0) - ((x.powf(2.0) / a.powf(2.0)) * b.powf(2.0)));
        for i in 0..(POINTS / 2) {
            let x = -a + (i as f64 * step);
            points.push(vec2(x, y(x)));
        }
        points.push(vec2(a, y(a)));
        for i in (1..(points.len() - 1)).rev() {
            let p = &points[i];
            points.push(vec2(p.x, -p.y));
        }
        points
            .iter_mut()
            .for_each(|p| *p = p.rotate(self.orientation));
        points.iter_mut().for_each(|p| *p += self.centre);
        for pair in points.windows(2) {
            let (p1, p2) = (&pair[0], &pair[1]);
            draw::line(p1, p2, colour);
        }
        draw::line(points.first().unwrap(), points.last().unwrap(), colour);
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_contains() {
        let ellipse = Ellipse::new(&vec2(0.0, 0.0), 0.0, 5.0, 10.0);
        assert!(ellipse.contains(&vec2(4.0, 0.0)));
        assert!(ellipse.contains(&vec2(0.0, 2.0)));
        assert!(!ellipse.contains(&vec2(0.0, 3.0)));
        assert!(!ellipse.contains(&vec2(0.0, -6.0)));
        assert!(!ellipse.contains(&vec2(5.0, -6.0)));
        assert!(!ellipse.contains(&vec2(0.0, 6.0)));
        assert!(!ellipse.contains(&vec2(6.0, 0.0)));
        let ellipse = Ellipse::new(&vec2(0.0, 0.0), std::f64::consts::FRAC_PI_4, 5.0, 10.0);
        assert!(ellipse.contains(&vec2(3.0, 3.0)));
        assert!(!ellipse.contains(&vec2(-3.0, 3.0)));
    }
    #[test]
    fn test_max_distance_to() {
        let ellipse = Ellipse::new(&vec2(0.0, 5.0), 0.0, 5.0, 10.0);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 0.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 10.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&vec2(5.0, 5.0)), 10.0);
        assert_eq!(ellipse.max_distance_to(&vec2(-5.0, 5.0)), 10.0);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 5.0)), 5.0);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 6.0)), 3.5);
        let height = f64::sqrt(5.0_f64.powi(2) + 5.0_f64.powi(2)) * 2.0;
        let ellipse = Ellipse::new(&vec2(5.0, 5.0), std::f64::consts::FRAC_PI_4, 50.0, height);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 0.0)), height);
        assert_eq!(ellipse.max_distance_to(&vec2(10.0, 10.0)), height);
        let angle = std::f64::consts::FRAC_PI_4 + std::f64::consts::FRAC_PI_2;
        let ellipse = Ellipse::new(&vec2(5.0, 5.0), angle, 50.0, height);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 10.0)), height);
    }
    #[test]
    fn test_min_distance_to() {
        let ellipse = Ellipse::new(&vec2(5.0, 0.0), std::f64::consts::FRAC_PI_2, 5.0, 10.0);
        assert_eq!(ellipse.min_distance_to(&vec2(0.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(10.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(5.0, 5.0)), 0.0);
        assert_eq!(ellipse.min_distance_to(&vec2(5.0, -5.0)), 0.0);
        assert_eq!(ellipse.min_distance_to(&vec2(5.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(5.5, 0.0)), 2.0);
    }
}
}
pub mod line {
use oort_api::prelude::*;
use super::{
    draw::{self, Colour},
    kinematics::Position,
};
#[derive(Clone, PartialEq, Debug)]
pub struct Line {
    pub points: (Vec2, Vec2),
}
impl Line {
    pub fn new<T: Position>(point1: &T, point2: &T) -> Self {
        return Self {
            points: (point1.position(), point2.position()),
        };
    }
}
impl<T: Position> From<(&T, &T)> for Line {
    fn from(points: (&T, &T)) -> Self {
        return Self {
            points: (points.0.position(), points.1.position()),
        };
    }
}
impl<T: Position> From<&(&T, &T)> for Line {
    fn from(points: &(&T, &T)) -> Self {
        return Self {
            points: (points.0.position(), points.1.position()),
        };
    }
}
impl Line {
    pub fn draw(&self, colour: Colour) {
        draw::line(&self.points.0, &self.points.1, colour);
    }
}
}
pub mod polygon {
use std::mem::MaybeUninit;
use oort_api::prelude::Vec2;
use super::{
    draw::{self, Colour},
    kinematics::Position,
};
#[derive(Clone, PartialEq, Debug)]
pub struct Polygon<const VERTICES: usize> {
    verticies: [Vec2; VERTICES],
}
impl<const V: usize, T: Position> From<[T; V]> for Polygon<V> {
    fn from(verticies: [T; V]) -> Self {
        let mut poly_verts: [MaybeUninit<Vec2>; V] = unsafe { MaybeUninit::uninit().assume_init() };
        for (from, to) in std::iter::zip(&verticies, &mut poly_verts) {
            to.write(from.position());
        }
        return Self {
            verticies: poly_verts.map(|v| unsafe { v.assume_init() }),
        };
    }
}
impl<const V: usize, T: Position> From<&[T; V]> for Polygon<V> {
    fn from(verticies: &[T; V]) -> Self {
        let mut poly_verts: [MaybeUninit<Vec2>; V] = unsafe { MaybeUninit::uninit().assume_init() };
        for (from, to) in std::iter::zip(verticies, &mut poly_verts) {
            to.write(from.position());
        }
        return Self {
            verticies: poly_verts.map(|v| unsafe { v.assume_init() }),
        };
    }
}
impl<const V: usize> Polygon<V> {
    pub fn get_verticies(&self) -> &[Vec2; V] {
        return &self.verticies;
    }
}
impl<const V: usize> Polygon<V> {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        if V < 3 {
            return false;
        }
        let mut wind_number = 0;
        for points in self.verticies.windows(2) {
            let line = (&points[0], &points[1]);
            if line.0.y <= point.position().y {
                if line.1.y > point.position().y && is_left_of_line(point, line) {
                    wind_number += 1;
                }
            } else if line.1.y <= point.position().y && is_right_of_line(point, line) {
                wind_number -= 1;
            }
        }
        let line = (
            unsafe { self.verticies.last().unwrap_unchecked() },
            unsafe { self.verticies.first().unwrap_unchecked() },
        );
        if line.0.y <= point.position().y {
            if line.1.y > point.position().y && is_left_of_line(point, line) {
                wind_number += 1;
            }
        } else if line.1.y <= point.position().y && is_right_of_line(point, line) {
            wind_number -= 1;
        }
        return wind_number != 0;
    }
}
fn is_left_of_line<T: Position>(point: &T, line: (&Vec2, &Vec2)) -> bool {
    return ((line.1.x - line.0.x) * (point.position().y - line.0.y)
        - (point.position().x - line.0.x) * (line.1.y - line.0.y))
        > 0.0;
}
fn is_right_of_line<T: Position>(point: &T, line: (&Vec2, &Vec2)) -> bool {
    return ((line.1.x - line.0.x) * (point.position().y - line.0.y)
        - (point.position().x - line.0.x) * (line.1.y - line.0.y))
        < 0.0;
}
impl<const V: usize> Polygon<V> {
    pub fn minmax_distance_to<T: Position>(&self, point: &T) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = 0.0;
        for vert in self.verticies {
            let dist = vert.distance_to(point);
            if dist < min {
                min = dist;
            }
            if dist > max {
                max = dist;
            }
        }
        return (min, max);
    }
    pub fn min_distance_to<T: Position>(&self, point: &T) -> f64 {
        let mut min = f64::INFINITY;
        for vert in self.verticies {
            let dist = vert.distance_to(&point.position());
            if dist < min {
                min = dist;
            }
        }
        return min;
    }
    pub fn max_distance_to<T: Position>(&self, point: &T) -> f64 {
        let mut max = 0.0;
        for vert in self.verticies {
            let dist = vert.distance_to(&point.position());
            if dist > max {
                max = dist;
            }
        }
        return max;
    }
}
impl<const V: usize> Polygon<V> {
    pub fn draw(&self, colour: Colour) {
        if V < 2 {
            return;
        }
        for points in self.verticies.windows(2) {
            draw::line(&points[0], &points[1], colour);
        }
        if V >= 3 {
            draw::line(
                self.verticies.first().unwrap(),
                self.verticies.last().unwrap(),
                colour,
            );
        }
    }
}
#[cfg(test)]
mod tests {
    use oort_api::prelude::vec2;
    use super::*;
    #[test]
    fn test_contains() {
        let poly: Polygon<4> = Polygon::from([
            vec2(2.0, 2.0),
            vec2(2.0, -2.0),
            vec2(-2.0, -2.0),
            vec2(-2.0, 2.0),
        ]);
        assert!(poly.contains(&vec2(0.0, 0.0)));
        assert!(poly.contains(&vec2(-1.0, -1.0)));
        assert!(poly.contains(&vec2(1.0, 1.0)));
        assert!(!poly.contains(&vec2(6.0, 0.0)));
        assert!(!poly.contains(&vec2(-6.0, 0.0)));
        assert!(!poly.contains(&vec2(-6.0, 5.0)));
        assert!(!poly.contains(&vec2(2.0, 0.0)));
        assert!(!poly.contains(&vec2(0.0, 2.0)));
    }
}
}
use super::{draw, kinematics};
pub use self::{circle::Circle, ellipse::Ellipse, line::Line, polygon::Polygon};
}
pub mod intercept {
use oort_api::prelude::*;
use super::{
    kinematics::{Acceleration, Position},
    polynomial::{self, Roots},
};
pub struct Intercept {
    time: f64,
    point: Vec2,
}
impl Position for Intercept {
    fn position(&self) -> Vec2 {
        return self.point;
    }
}
impl Intercept {
    pub fn new<T: Acceleration, U: Acceleration>(
        vessel: &T,
        acceleration: f64,
        target: &U,
    ) -> Self {
        let relpos = target.position_relative_to(vessel);
        let relvel = target.velocity_relative_to(vessel);
        let a = 0.25 * (acceleration.powf(2.0) - target.acceleration_magnitude().powf(2.0));
        let b = -relvel.dot(target.acceleration());
        let c = -target.acceleration().dot(relpos) - relvel.dot(relvel);
        let d = -2.0 * relpos.dot(relvel);
        let e = -relpos.dot(relpos);
        let roots = polynomial::find_roots_quartic(a, b, c, d, e);
        let time = if let Roots::Four(roots) = roots {
            debug!("4 roots: {:?}", roots);
            roots[1]
        } else if let Roots::Two(roots) = roots {
            debug!("2 roots: {:?}", roots);
            roots[1]
        } else {
            debug!("? roots");
            f64::NAN
        };
        let point =
            target.position() + (relvel * time) + (0.5 * target.acceleration() * time.powf(2.0));
        return Self { time, point };
    }
}
}
pub mod kinematics {
use std::f64::consts::PI;
use oort_api::prelude::*;
pub struct Circle {
    pub centre: Vec2,
    pub radius: f64,
}
impl Position for Circle {
    fn position(&self) -> Vec2 {
        return self.centre;
    }
}
pub struct KinematicModel {
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
}
impl Position for KinematicModel {
    fn position(&self) -> Vec2 {
        return self.position;
    }
}
impl Velocity for KinematicModel {
    fn velocity(&self) -> Vec2 {
        return self.velocity;
    }
}
impl Acceleration for KinematicModel {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl<T: Acceleration> From<&T> for KinematicModel {
    fn from(value: &T) -> Self {
        return Self {
            position: value.position(),
            velocity: value.velocity(),
            acceleration: value.acceleration(),
        };
    }
}
pub trait Position {
    fn position(&self) -> Vec2;
    fn position_relative_to<T: Position>(&self, other: &T) -> Vec2 {
        return self.position() - other.position();
    }
    fn distance_to<T: Position>(&self, other: &T) -> f64 {
        return self.position_relative_to(other).length();
    }
    fn bearing_to<T: Position>(&self, other: &T) -> f64 {
        let relative_position = other.position() - self.position();
        return relative_position.angle();
    }
}
impl Position for Vec2 {
    fn position(&self) -> Vec2 {
        return *self;
    }
}
pub trait Velocity: Position {
    fn velocity(&self) -> Vec2;
    fn velocity_relative_to<T: Velocity>(&self, other: &T) -> Vec2 {
        return self.velocity() - other.velocity();
    }
    fn speed(&self) -> f64 {
        return self.velocity().length();
    }
    fn speed_relative_to<T: Velocity>(&self, other: &T) -> f64 {
        return (self.velocity() - other.velocity()).length();
    }
    fn orbital_velocity_to<T: Velocity>(&self, other: &T) -> f64 {
        let pos = self.position_relative_to(other);
        let theta = angle_diff(pos.angle(), self.velocity_relative_to(other).angle());
        return (self.speed_relative_to(other) * f64::sin(theta)) / self.distance_to(other);
    }
    fn possible_position_after(&self, max_acceleration: f64, seconds: f64) -> Circle {
        let centre = self.position() + (self.velocity() * seconds);
        let radius = 0.5 * max_acceleration * seconds.powf(2.0);
        return Circle { centre, radius };
    }
}
pub trait Acceleration: Velocity {
    fn acceleration(&self) -> Vec2;
    fn acceleration_magnitude(&self) -> f64 {
        return self.acceleration().length();
    }
    fn acceleration_relative_to<T: Acceleration>(&self, other: &T) -> Vec2 {
        return self.acceleration() - other.acceleration();
    }
    fn position_after(&self, seconds: f64) -> Vec2 {
        return self.position()
            + (self.velocity() * seconds)
            + 0.5 * self.acceleration() * seconds.powf(2.0);
    }
    fn velocity_after(&self, seconds: f64) -> Vec2 {
        return self.velocity() + (self.acceleration() * seconds);
    }
    fn orbital_acceleration_to<T: Position>(&self, other: &T) -> f64 {
        let vector_prograde = self.position_relative_to(other).rotate(-PI / 4.0);
        let angle_prograde = angle_diff(vector_prograde.angle(), self.acceleration().angle());
        let acceleration_prograde = self.acceleration_magnitude() * f64::cos(angle_prograde);
        let vector_radial = self.position_relative_to(other);
        let angle_radial = angle_diff(vector_radial.angle(), self.acceleration().angle());
        let velocity_radial = self.speed() * f64::cos(angle_radial);
        let velocity_prograde = self.speed() * f64::cos(angle_prograde);
        return (self.distance_to(other) * acceleration_prograde)
            + (2.0 * velocity_radial * velocity_prograde);
    }
}
pub trait Heading: Position {
    fn heading(&self) -> f64;
    fn relative_bearing_to<T: Position>(&self, target: &T) -> f64 {
        let target_vector = target.position() - self.position();
        return angle_diff(self.heading(), target_vector.angle());
    }
}
pub trait AngularVelocity: Heading {
    fn angular_velocity(&self) -> f64;
    fn heading_after(&self, seconds: f64) -> f64 {
        let heading = self.heading() + PI; // Normalise between 0 and 2 * PI.
        let heading = heading + (self.angular_velocity() * seconds);
        let heading = heading % (2.0 * PI); // Wrap.
        return heading - PI;
    }
}
#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use oort_api::prelude::*;
    use super::{AngularVelocity, Heading, Position};
    struct TestVessel {
        pub position: Vec2,
        pub heading: f64,
        pub angular_velocity: f64,
    }
    impl Position for TestVessel {
        fn position(&self) -> oort_api::prelude::Vec2 {
            return self.position;
        }
    }
    impl Heading for TestVessel {
        fn heading(&self) -> f64 {
            return self.heading;
        }
    }
    impl AngularVelocity for TestVessel {
        fn angular_velocity(&self) -> f64 {
            return self.angular_velocity;
        }
    }
    #[test]
    fn test_heading_after() {
        let vessel = TestVessel {
            position: vec2(0.0, 0.0),
            heading: 0.0,
            angular_velocity: PI / 8.0,
        };
        assert_eq!(vessel.heading_after(1.0), PI / 8.0);
        assert_eq!(vessel.heading_after(8.0), -PI);
        assert_eq!(vessel.heading_after(16.0), 0.0);
    }
}
}
pub mod polynomial {
use oort_api::prelude::*;
use std::f32;
use std::f64;
use std::fmt::Debug;
use std::ops::Add;
use std::ops::Div;
use std::ops::Mul;
use std::ops::Neg;
use std::ops::Sub;
pub trait FloatType:
    Sized
    + Copy
    + Debug
    + From<i16>
    + PartialEq
    + PartialOrd
    + Neg<Output = Self>
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
{
    fn zero() -> Self;
    fn one() -> Self;
    fn one_third() -> Self;
    fn pi() -> Self;
    fn two_third_pi() -> Self;
    fn sqrt(self) -> Self;
    fn cbrt(self) -> Self {
        if self < Self::zero() {
            -(-self).powf(Self::one_third())
        } else {
            self.powf(Self::one_third())
        }
    }
    fn atan(self) -> Self;
    fn acos(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn abs(self) -> Self;
    fn powf(self, n: Self) -> Self;
}
impl FloatType for f32 {
    fn zero() -> Self {
        0f32
    }
    fn one_third() -> Self {
        1f32 / 3f32
    }
    fn one() -> Self {
        1f32
    }
    fn two_third_pi() -> Self {
        2f32 * f32::consts::FRAC_PI_3
    }
    fn pi() -> Self {
        f32::consts::PI
    }
    fn sqrt(self) -> Self {
        self.sqrt()
    }
    fn atan(self) -> Self {
        self.atan()
    }
    fn acos(self) -> Self {
        self.acos()
    }
    fn sin(self) -> Self {
        self.sin()
    }
    fn cos(self) -> Self {
        self.cos()
    }
    fn abs(self) -> Self {
        self.abs()
    }
    fn powf(self, n: Self) -> Self {
        self.powf(n)
    }
}
impl FloatType for f64 {
    fn zero() -> Self {
        0f64
    }
    fn one_third() -> Self {
        1f64 / 3f64
    }
    fn one() -> Self {
        1f64
    }
    fn two_third_pi() -> Self {
        2f64 * f64::consts::FRAC_PI_3
    }
    fn pi() -> Self {
        f64::consts::PI
    }
    fn sqrt(self) -> Self {
        self.sqrt()
    }
    fn atan(self) -> Self {
        self.atan()
    }
    fn acos(self) -> Self {
        self.acos()
    }
    fn sin(self) -> Self {
        self.sin()
    }
    fn cos(self) -> Self {
        self.cos()
    }
    fn abs(self) -> Self {
        self.abs()
    }
    fn powf(self, n: Self) -> Self {
        self.powf(n)
    }
}
#[derive(Debug, PartialEq)]
pub enum Roots {
    None([f64; 0]),
    One([f64; 1]),
    Two([f64; 2]),
    Three([f64; 3]),
    Four([f64; 4]),
}
impl AsRef<[f64]> for Roots {
    fn as_ref(&self) -> &[f64] {
        match *self {
            Roots::None(ref x) => x,
            Roots::One(ref x) => x,
            Roots::Two(ref x) => x,
            Roots::Three(ref x) => x,
            Roots::Four(ref x) => x,
        }
    }
}
impl Roots {
    fn check_new_root(&self, new_root: f64) -> (bool, usize) {
        let mut pos = 0;
        let mut exists = false;
        for x in self.as_ref().iter() {
            if *x == new_root {
                exists = true;
                break;
            }
            if *x > new_root {
                break;
            }
            pos += 1;
        }
        (exists, pos)
    }
    pub fn add_new_root(self, new_root: f64) -> Self {
        match self {
            Roots::None(_) => Roots::One([new_root]),
            _ => {
                let (exists, pos) = self.check_new_root(new_root);
                if exists {
                    self
                } else {
                    let old_roots = self.as_ref();
                    match (old_roots.len(), pos) {
                        (1, 0) => Roots::Two([new_root, old_roots[0]]),
                        (1, 1) => Roots::Two([old_roots[0], new_root]),
                        (2, 0) => Roots::Three([new_root, old_roots[0], old_roots[1]]),
                        (2, 1) => Roots::Three([old_roots[0], new_root, old_roots[1]]),
                        (2, 2) => Roots::Three([old_roots[0], old_roots[1], new_root]),
                        (3, 0) => Roots::Four([new_root, old_roots[0], old_roots[1], old_roots[2]]),
                        (3, 1) => Roots::Four([old_roots[0], new_root, old_roots[1], old_roots[2]]),
                        (3, 2) => Roots::Four([old_roots[0], old_roots[1], new_root, old_roots[2]]),
                        (3, 3) => Roots::Four([old_roots[0], old_roots[1], old_roots[2], new_root]),
                        _ => panic!("Cannot add root"),
                    }
                }
            }
        }
    }
}
pub fn find_roots_linear(a1: f64, a0: f64) -> Roots {
    if a1 == 0.0 {
        if a0 == 0.0 {
            Roots::One([0.0])
        } else {
            Roots::None([])
        }
    } else {
        Roots::One([-a0 / a1])
    }
}
pub fn find_roots_quadratic(a2: f64, a1: f64, a0: f64) -> Roots {
    if a2 == 0.0 {
        find_roots_linear(a1, a0)
    } else {
        let discriminant = a1 * a1 - 4.0 * a2 * a0;
        if discriminant < 0.0 {
            Roots::None([])
        } else {
            let a2x2 = 2.0 * a2;
            if discriminant == 0.0 {
                Roots::One([-a1 / a2x2])
            } else {
                let sq = discriminant.sqrt();
                let (same_sign, diff_sign) = if a1 < 0.0 {
                    (-a1 + sq, -a1 - sq)
                } else {
                    (-a1 - sq, -a1 + sq)
                };
                let (x1, x2) = if same_sign.abs() > a2x2.abs() {
                    let a0x2 = 2.0 * a0;
                    if diff_sign.abs() > a2x2.abs() {
                        (a0x2 / same_sign, a0x2 / diff_sign)
                    } else {
                        (a0x2 / same_sign, same_sign / a2x2)
                    }
                } else {
                    (diff_sign / a2x2, same_sign / a2x2)
                };
                if x1 < x2 {
                    Roots::Two([x1, x2])
                } else {
                    Roots::Two([x2, x1])
                }
            }
        }
    }
}
pub fn find_roots_cubic_normalized(a2: f64, a1: f64, a0: f64) -> Roots {
    let q = (3.0 * a1 - a2 * a2) / 9.0;
    let r = (9.0 * a2 * a1 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0;
    let q3 = q * q * q;
    let d = q3 + r * r;
    let a2_div_3 = a2 / 3.0;
    if d < 0.0 {
        let phi_3 = (r / (-q3).sqrt()).acos() / 3.0;
        let sqrt_q_2 = 2.0 * (-q).sqrt();
        Roots::One([sqrt_q_2 * phi_3.cos() - a2_div_3])
            .add_new_root(sqrt_q_2 * (phi_3 - (2.0 * f64::consts::FRAC_PI_3)).cos() - a2_div_3)
            .add_new_root(sqrt_q_2 * (phi_3 + (2.0 * f64::consts::FRAC_PI_3)).cos() - a2_div_3)
    } else {
        let sqrt_d = d.sqrt();
        let s = (r + sqrt_d).cbrt();
        let t = (r - sqrt_d).cbrt();
        if s == t {
            if s + t == 0.0 {
                Roots::One([s + t - a2_div_3])
            } else {
                Roots::One([s + t - a2_div_3]).add_new_root(-(s + t) / 2.0 - a2_div_3)
            }
        } else {
            Roots::One([s + t - a2_div_3])
        }
    }
}
pub fn find_roots_quartic_depressed(a2: f64, a1: f64, a0: f64) -> Roots {
    if a1 == 0.0 {
        debug!("Unhandled depressed quartic form: a1 == 0");
        return Roots::None([]);
    } else if a0 == 0.0 {
        debug!("Unhandled depressed quartic form: a0 == 0");
        return Roots::None([]);
    } else {
        let a2_pow_2 = a2 * a2;
        let a1_div_2 = a1 / 2.0;
        let b2 = a2 * 5.0 / 2.0;
        let b1 = 2.0 * a2_pow_2 - a0;
        let b0 = (a2_pow_2 * a2 - a2 * a0 - a1_div_2 * a1_div_2) / 2.0;
        let resolvent_roots = find_roots_cubic_normalized(b2, b1, b0);
        let y = resolvent_roots.as_ref().iter().last().unwrap();
        let _a2_plus_2y = a2 + 2.0 * *y;
        if _a2_plus_2y > 0.0 {
            let sqrt_a2_plus_2y = _a2_plus_2y.sqrt();
            let q0a = a2 + *y - a1_div_2 / sqrt_a2_plus_2y;
            let q0b = a2 + *y + a1_div_2 / sqrt_a2_plus_2y;
            let mut roots = find_roots_quadratic(1.0, sqrt_a2_plus_2y, q0a);
            for x in find_roots_quadratic(1.0, -sqrt_a2_plus_2y, q0b)
                .as_ref()
                .iter()
            {
                roots = roots.add_new_root(*x);
            }
            roots
        } else {
            Roots::None([])
        }
    }
}
fn find_roots_via_depressed_quartic(
    a4: f64,
    a3: f64,
    a2: f64,
    a1: f64,
    a0: f64,
    pp: f64,
    rr: f64,
    dd: f64,
) -> Roots {
    let a4_pow_2 = a4 * a4;
    let a4_pow_3 = a4_pow_2 * a4;
    let a4_pow_4 = a4_pow_2 * a4_pow_2;
    let p = pp / (8.0 * a4_pow_2);
    let q = rr / (8.0 * a4_pow_3);
    let r =
        (dd + 16.0 * a4_pow_2 * (12.0 * a0 * a4 - 3.0 * a1 * a3 + a2 * a2)) / (256.0 * a4_pow_4);
    let mut roots = Roots::None([]);
    for y in find_roots_quartic_depressed(p, q, r).as_ref().iter() {
        roots = roots.add_new_root(*y - a3 / (4.0 * a4));
    }
    roots
}
pub fn find_roots_quartic(a4: f64, a3: f64, a2: f64, a1: f64, a0: f64) -> Roots {
    if a4 == 0.0 {
        debug!("Unhandled quartic form: a4 == 0");
        return Roots::None([]);
    } else if a0 == 0.0 {
        debug!("Unhandled quartic form: a0 == 0");
        return Roots::None([]);
    } else if a1 == 0.0 && a3 == 0.0 {
        debug!("Unhandled quartic form: a1 && a3 == 0");
        return Roots::None([]);
    } else {
        let discriminant =
            a4 * a0 * a4 * (256.0 * a4 * a0 * a0 + a1 * (144.0 * a2 * a1 - 192.0 * a3 * a0))
                + a4 * a0 * a2 * a2 * (16.0 * a2 * a2 - 80.0 * a3 * a1 - 128.0 * a4 * a0)
                + (a3
                    * a3
                    * (a4 * a0 * (144.0 * a2 * a0 - 6.0 * a1 * a1)
                        + (a0 * (18.0 * a3 * a2 * a1 - 27.0 * a3 * a3 * a0 - 4.0 * a2 * a2 * a2)
                            + a1 * a1 * (a2 * a2 - 4.0 * a3 * a1))))
                + a4 * a1 * a1 * (18.0 * a3 * a2 * a1 - 27.0 * a4 * a1 * a1 - 4.0 * a2 * a2 * a2);
        let pp = 8.0 * a4 * a2 - 3.0 * a3 * a3;
        let rr = a3 * a3 * a3 + 8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2;
        let delta0 = a2 * a2 - 3.0 * a3 * a1 + 12.0 * a4 * a0;
        let dd = 64.0 * a4 * a4 * a4 * a0 - 16.0 * a4 * a4 * a2 * a2 + 16.0 * a4 * a3 * a3 * a2
            - 16.0 * a4 * a4 * a3 * a1
            - 3.0 * a3 * a3 * a3 * a3;
        let double_root = discriminant == 0.0;
        if double_root {
            let triple_root = double_root && delta0 == 0.0;
            let quadruple_root = triple_root && dd == 0.0;
            let no_roots = dd == 0.0 && pp > 0.0 && rr == 0.0;
            if quadruple_root {
                Roots::One([-a3 / (4.0 * a4)])
            } else if triple_root {
                let x0 = (-72.0 * a4 * a4 * a0 + 10.0 * a4 * a2 * a2 - 3.0 * a3 * a3 * a2)
                    / (9.0 * (8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2 + a3 * a3 * a3));
                let roots = Roots::One([x0]);
                roots.add_new_root(-(a3 / a4 + 3.0 * x0))
            } else if no_roots {
                Roots::None([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            }
        } else {
            let no_roots = discriminant > 0.0 && (pp > 0.0 || dd > 0.0);
            return if no_roots {
                Roots::None([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            };
        }
    }
}
}
use super::draw;
pub use self::{firing_solution::FiringSolution, intercept::Intercept};
}
pub mod radar {
pub mod common {
use oort_api::prelude::*;
pub trait Radar {
    const MAX_RADAR_RANGE: f64 = 25000.0;
    fn set_heading(&self, heading: f64) {
        set_radar_heading(heading);
    }
    fn get_heading(&self) -> f64 {
        return radar_heading();
    }
    fn set_width(&self, heading: f64) {
        set_radar_width(heading);
    }
    fn get_width(&self) -> f64 {
        return radar_width();
    }
    fn set_min_distance(&self, distance: f64) {
        set_radar_min_distance(distance);
    }
    fn get_min_distance(&self) -> f64 {
        return radar_min_distance();
    }
    fn set_max_distance(&self, distance: f64) {
        set_radar_max_distance(distance);
    }
    fn get_max_distance(&self) -> f64 {
        return radar_max_distance();
    }
    fn get_scan(&self) -> Option<ScanResult> {
        return scan();
    }
}
}
pub mod contacts {
pub mod board {
use std::collections::{btree_map::Iter, BTreeMap};
use oort_api::debug;
use super::{contact::Contact, draw::Colour, track::TrackedContact};
#[derive(Clone, PartialEq, Debug)]
pub struct ContactBoard {
    contacts: BTreeMap<usize, Contact>,
}
impl ContactBoard {
    pub fn new() -> Self {
        return Self {
            contacts: BTreeMap::new(),
        };
    }
}
impl ContactBoard {
    pub fn add(&mut self, contact: Contact) {
        match contact {
            Contact::Scanned(contact) => self.add_search_contact(Contact::Scanned(contact)),
            Contact::Tracked(contact) => self.add_tracked_contact(Contact::Tracked(contact)),
        }
    }
    pub fn remove(&mut self, id: usize) {
        self.contacts.remove(&id);
    }
    fn add_search_contact(&mut self, contact: Contact) {
        let contact_area = contact.get_area_after(contact.time_elapsed());
        let matches = self.iter();
        let matches = matches.filter(|(_, c)| c.is_class(contact.get_class()));
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches =
            matches.filter(|(_, c, area)| area.contains(&contact) || contact_area.contains(*c));
        if matches
            .clone()
            .any(|(_, c, _)| matches!(c, Contact::Tracked(_)))
        {
            return;
        }
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.contacts.remove(id);
        }
        let id = matches
            .first()
            .cloned()
            .or(self.contacts.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);
        self.contacts.insert(id, contact);
    }
    fn add_tracked_contact(&mut self, contact: Contact) {
        let matches = self.iter();
        let matches = matches.filter(|(_, c)| c.is_class(contact.get_class()));
        let matches = matches.map(|(id, c)| (id, c, c.get_area_after(c.time_elapsed())));
        let matches = matches.filter(|(_, _, area)| area.contains(&contact));
        let matches = matches.map(|(id, _, _)| *id);
        let matches = matches.collect::<Vec<usize>>();
        for id in matches.iter().skip(1) {
            self.contacts.remove(id);
        }
        let id = matches
            .first()
            .cloned()
            .or(self.contacts.last_key_value().map(|(k, _)| k + 1))
            .unwrap_or(0);
        self.contacts.insert(id, contact);
    }
}
impl ContactBoard {
    pub fn insert(&mut self, id: usize, contact: Contact) {
        self.contacts.insert(id, contact);
    }
    pub fn remove_matching(&mut self, contact: &Contact) {
        let matching_class = self
            .contacts
            .iter()
            .filter(|(_, c)| c.is_class(contact.get_class()));
        let matches =
            matching_class.filter(|(_, c)| c.get_area_after(c.time_elapsed()).contains(contact));
        let matches = matches.filter(|(_, c)| matches!(c, Contact::Scanned(_)));
        let match_ids: Vec<usize> = matches.map(|(id, _)| *id).collect();
        for id in match_ids {
            self.contacts.remove(&id);
        }
    }
    fn find_matching_contacts(&self, contact: &Contact) -> Vec<usize> {
        let mut matches = Vec::new();
        let matching_class = self
            .contacts
            .iter()
            .filter(|(_, c)| c.is_class(contact.get_class()));
        for (&i, old) in matching_class {
            if old.get_area_after(old.time_elapsed()).contains(contact) {
                matches.push(i);
            }
        }
        return matches;
    }
    pub fn iter(&self) -> Iter<'_, usize, Contact> {
        return self.contacts.iter();
    }
    pub fn track(&mut self, id: usize) {
        if let Some(Contact::Scanned(contact)) = self.contacts.get(&id) {
            self.contacts
                .insert(id, Contact::Tracked(TrackedContact::from(contact)));
        }
    }
    pub fn get(&self, id: usize) -> Option<&Contact> {
        return self.contacts.get(&id);
    }
    pub fn get_mut(&mut self, id: usize) -> Option<&mut Contact> {
        return self.contacts.get_mut(&id);
    }
    pub fn take(&mut self, id: usize) -> Option<Contact> {
        return self.contacts.remove(&id);
    }
    pub fn count(&self) -> usize {
        return self.contacts.len();
    }
    pub fn draw_contacts(&self) {
        for (_, contact) in self.contacts.iter() {
            let colour = match contact {
                Contact::Scanned(_) => Colour::Red,
                Contact::Tracked(_) => Colour::Green,
            };
            contact.get_area_after(contact.time_elapsed()).draw(colour);
        }
    }
}
}
pub mod contact {
use oort_api::prelude::{Class, Vec2};
use super::super::super::math::kinematics::Position;
use super::{
    math::geometry::{Ellipse, Polygon},
    SearchContact, TrackedContact,
};
#[derive(Clone, PartialEq, Debug)]
pub enum Contact {
    Scanned(SearchContact),
    Tracked(TrackedContact),
}
impl Position for Contact {
    fn position(&self) -> Vec2 {
        return match self {
            Self::Scanned(contact) => contact.position(),
            Self::Tracked(contact) => contact.position(),
        };
    }
}
impl Contact {
    pub fn time_elapsed(&self) -> f64 {
        return match self {
            Self::Scanned(contact) => contact.time_elapsed(),
            Self::Tracked(contact) => contact.time_elapsed(),
        };
    }
    pub fn is_class(&self, class: Class) -> bool {
        return match self {
            Self::Scanned(contact) => contact.class == class,
            Self::Tracked(contact) => contact.class == class,
        };
    }
    pub fn get_class(&self) -> Class {
        return match self {
            Self::Scanned(contact) => contact.class,
            Self::Tracked(contact) => contact.class,
        };
    }
    pub fn get_area_after(&self, time: f64) -> Ellipse {
        return match self {
            Self::Scanned(contact) => contact.get_area_after(time),
            Self::Tracked(contact) => contact.get_area_after(time),
        };
    }
}
}
pub mod error {
use oort_api::prelude::ScanResult;
const BEARING_NOISE_FACTOR: f64 = 1e1 * (std::f64::consts::TAU / 360.0);
const DISTANCE_NOISE_FACTOR: f64 = 1e4;
const VELOCITY_NOISE_FACTOR: f64 = 1e2;
const RNG_RANGE: f64 = 4.0;
#[derive(Clone, PartialEq, Debug)]
pub struct RadarContactError {
    pub bearing: f64,
    pub distance: f64,
    pub velocity: f64,
}
impl From<ScanResult> for RadarContactError {
    fn from(scan: ScanResult) -> Self {
        let error_factor = 10.0_f64.powf(-scan.snr / 10.0);
        return Self {
            bearing: BEARING_NOISE_FACTOR * error_factor * RNG_RANGE,
            distance: DISTANCE_NOISE_FACTOR * error_factor * RNG_RANGE,
            velocity: VELOCITY_NOISE_FACTOR * error_factor * RNG_RANGE,
        };
    }
}
impl From<&ScanResult> for RadarContactError {
    fn from(scan: &ScanResult) -> Self {
        let error_factor = 10.0_f64.powf(-scan.snr / 10.0);
        return Self {
            bearing: BEARING_NOISE_FACTOR * error_factor * RNG_RANGE,
            distance: DISTANCE_NOISE_FACTOR * error_factor * RNG_RANGE,
            velocity: VELOCITY_NOISE_FACTOR * error_factor * RNG_RANGE,
        };
    }
}
}
pub mod search {
use oort_api::prelude::*;
use super::{
    emitter::Emitter,
    error::RadarContactError,
    math::{
        geometry::Ellipse,
        kinematics::{Position, Velocity},
    },
    ship::stats::MaxAcceleration,
};
#[derive(Clone, PartialEq, Debug)]
pub struct SearchContact {
    pub time: f64,
    pub emitter: Emitter,
    pub class: Class,
    pub position: Vec2,
    pub velocity: Vec2,
    pub rssi: f64,
    pub snr: f64,
    pub error: RadarContactError,
}
impl SearchContact {
    pub fn new(time: f64, emitter: &Emitter, scan: &ScanResult) -> Self {
        Self {
            time,
            emitter: emitter.clone(),
            class: scan.class,
            position: scan.position,
            velocity: scan.velocity,
            rssi: scan.rssi,
            snr: scan.snr,
            error: RadarContactError::from(scan),
        }
    }
}
impl Position for SearchContact {
    fn position(&self) -> Vec2 {
        return self.position;
    }
}
impl Velocity for SearchContact {
    fn velocity(&self) -> Vec2 {
        return self.velocity;
    }
}
impl SearchContact {
    pub fn time_elapsed(&self) -> f64 {
        return current_time() - self.time;
    }
    pub fn is_same_class(&self, other: &Self) -> bool {
        return self.class == other.class;
    }
}
impl SearchContact {
    pub fn get_area_after(&self, time: f64) -> Ellipse {
        let mut area = self.get_initial_area();
        area.translate(&(self.velocity * time));
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(self.error.velocity * time + (0.5 * max_accel.magnitude() * time.powf(2.0)));
        return area;
    }
    pub fn get_initial_area(&self) -> Ellipse {
        let bearing = self.bearing_to(&self.emitter.position);
        let distance = self.distance_to(&self.emitter.position);
        let width = f64::atan(self.error.bearing) * distance * 2.0;
        let height = self.error.distance * 2.0;
        return Ellipse::new(&self.position, bearing, width, height);
    }
}
}
pub mod track {
use std::collections::VecDeque;
use oort_api::prelude::*;
use super::{
    emitter::Emitter,
    error::RadarContactError,
    math::{
        geometry::Ellipse,
        kinematics::{Acceleration, Position, Velocity},
    },
    ship::stats::MaxAcceleration,
    SearchContact,
};
#[derive(Clone, PartialEq, Debug)]
pub struct TrackedContact {
    emitter: VecDeque<Emitter>,
    timestamp: VecDeque<f64>,
    pub class: Class,
    position: VecDeque<Vec2>,
    velocity: VecDeque<Vec2>,
    rssi: VecDeque<f64>,
    snr: VecDeque<f64>,
    error: VecDeque<RadarContactError>,
    acceleration: Vec2,
}
impl TrackedContact {
    const MAX_DATA_POINTS: usize = 9;
    pub fn new(scan: &ScanResult, scan_emitter: &Emitter) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(scan_emitter.clone());
        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(current_time());
        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(scan.position);
        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(scan.velocity);
        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(scan.rssi);
        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(scan.snr);
        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(RadarContactError::from(scan));
        return Self {
            emitter,
            timestamp,
            class: scan.class,
            position,
            velocity,
            rssi,
            snr,
            error,
            acceleration: Vec2::default(),
        };
    }
}
impl From<SearchContact> for TrackedContact {
    fn from(contact: SearchContact) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(contact.emitter);
        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(contact.time);
        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(contact.position);
        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(contact.velocity);
        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(contact.rssi);
        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(contact.snr);
        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(contact.error);
        return Self {
            emitter,
            timestamp,
            class: contact.class,
            position,
            velocity,
            rssi,
            snr,
            error,
            acceleration: Vec2::default(),
        };
    }
}
impl From<&SearchContact> for TrackedContact {
    fn from(contact: &SearchContact) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(contact.emitter.clone());
        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(contact.time);
        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(contact.position);
        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(contact.velocity);
        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(contact.rssi);
        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(contact.snr);
        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(contact.error.clone());
        return Self {
            emitter,
            timestamp,
            class: contact.class,
            position,
            velocity,
            rssi,
            snr,
            error,
            acceleration: Vec2::default(),
        };
    }
}
impl Position for TrackedContact {
    fn position(&self) -> Vec2 {
        return *self.position.back().unwrap();
    }
}
impl Velocity for TrackedContact {
    fn velocity(&self) -> Vec2 {
        return *self.velocity.back().unwrap();
    }
}
impl Acceleration for TrackedContact {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl TrackedContact {
    pub fn update(&mut self, contact: ScanResult, emitter: &Emitter) {
        if self.emitter.len() == Self::MAX_DATA_POINTS {
            self.emitter.pop_front();
        }
        self.emitter.push_back(emitter.clone());
        if self.timestamp.len() == Self::MAX_DATA_POINTS {
            self.timestamp.pop_front();
        }
        self.timestamp.push_back(current_time());
        if self.position.len() == Self::MAX_DATA_POINTS {
            self.position.pop_front();
        }
        self.position.push_back(contact.position);
        if self.velocity.len() == Self::MAX_DATA_POINTS {
            self.velocity.pop_front();
        }
        self.velocity.push_back(contact.velocity);
        if self.rssi.len() == Self::MAX_DATA_POINTS {
            self.rssi.pop_front();
        }
        self.rssi.push_back(contact.rssi);
        if self.snr.len() == Self::MAX_DATA_POINTS {
            self.snr.pop_front();
        }
        self.snr.push_back(contact.snr);
        if self.error.len() == Self::MAX_DATA_POINTS {
            self.error.pop_front();
        }
        self.error.push_back(RadarContactError::from(contact));
        let mut sum = vec2(0.0, 0.0);
        let records = std::iter::zip(self.velocity.iter(), self.timestamp.iter());
        for (r1, r2) in std::iter::zip(records.clone(), records.skip(1)) {
            let vdelta = r1.0 - r2.0;
            let tdelta = r1.1 - r2.1;
            let accel = vdelta / tdelta;
            sum += accel;
        }
        self.acceleration = sum / (self.velocity.len() - 1) as f64;
    }
}
impl TrackedContact {
    pub fn time_elapsed(&self) -> f64 {
        return current_time() - self.timestamp.back().unwrap();
    }
    pub fn is_same_class(&self, other: &Self) -> bool {
        return self.class == other.class;
    }
}
impl TrackedContact {
    pub fn get_area_after(&self, time: f64) -> Ellipse {
        let mut area = self.get_initial_area();
        area.translate(&(*self.velocity.back().unwrap() * time));
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(
            self.error.back().unwrap().velocity * time
                + (0.5 * max_accel.magnitude() * time.powf(2.0)),
        );
        return area;
    }
    pub fn get_initial_area(&self) -> Ellipse {
        let emitter = self.emitter.back().unwrap();
        let bearing = self.bearing_to(&emitter.position);
        let distance = self.distance_to(&emitter.position);
        let error = self.error.back().unwrap();
        let max_distance = distance + error.distance;
        let min_distance = distance - error.distance;
        let max_bearing = bearing + error.bearing;
        let min_bearing = bearing - error.bearing;
        let max_distance = max_distance.clamp(emitter.min_distance, emitter.max_distance);
        let min_distance = min_distance.clamp(emitter.min_distance, emitter.max_distance);
        let max_bearing = max_bearing.clamp(emitter.get_min_heading(), emitter.get_max_heading());
        let min_bearing = min_bearing.clamp(emitter.get_min_heading(), emitter.get_max_heading());
        let width = f64::atan(self.error.back().unwrap().bearing) * distance * 2.0;
        let height = self.error.back().unwrap().distance * 2.0;
        return Ellipse::new(self.position.back().unwrap(), bearing, width, height);
    }
}
}
use super::{draw, emitter, math, ship};
pub use self::{
    board::ContactBoard, contact::Contact, search::SearchContact, track::TrackedContact,
};
}
pub mod emitter {
use oort_api::prelude::*;
use super::{common::Radar, math::kinematics::Position};
#[derive(Clone, PartialEq, Debug)]
pub struct EmitterRange {
    pub min: f64,
    pub max: f64,
}
#[derive(Clone, PartialEq, Debug)]
pub struct Emitter {
    pub position: Vec2,
    pub min_distance: f64,
    pub max_distance: f64,
    pub heading: f64,
    pub width: f64,
}
impl Emitter {
    pub fn new<T: Position, R: Radar>(emitter: &T, radar: &R) -> Self {
        return Self {
            position: emitter.position(),
            min_distance: radar.get_min_distance(),
            max_distance: radar.get_max_distance(),
            heading: radar.get_heading(),
            width: radar.get_width(),
        };
    }
}
impl Emitter {
    pub fn get_min_heading(&self) -> f64 {
        return self.heading - (self.width / 2.0);
    }
    pub fn get_max_heading(&self) -> f64 {
        return self.heading + (self.width / 2.0);
    }
}
}
pub mod manager {
use oort_api::prelude::debug;
use super::{
    contacts::{Contact, ContactBoard, TrackedContact},
    math::kinematics::{Acceleration, Position},
    search::ScanningRadar,
    track::TrackingRadar,
};
#[derive(Clone, PartialEq, Debug)]
pub enum RadarJob {
    Search,
    Track(usize),
}
#[derive(Clone, PartialEq, Debug)]
pub struct RadarManager {
    pub contacts: ContactBoard,
    job_rotation: Vec<RadarJob>,
    job_index: usize,
    search: ScanningRadar,
    track: TrackingRadar,
}
impl RadarManager {
    pub fn new() -> Self {
        return Self {
            contacts: ContactBoard::new(),
            job_rotation: vec![RadarJob::Search],
            job_index: 0,
            search: ScanningRadar::new(),
            track: TrackingRadar::new(),
        };
    }
}
impl RadarManager {
    pub fn scan<T: Position>(&mut self, emitter: &T) {
        match self.job_rotation.get(self.job_index) {
            Some(RadarJob::Search) => {
                if let Some(contact) = self.search.scan(emitter).map(Contact::Scanned) {
                    self.contacts.add(contact);
                }
            }
            Some(RadarJob::Track(id)) => {
                let updated_contact = self
                    .contacts
                    .take(*id)
                    .and_then(|c| self.track.scan(c, emitter))
                    .map(Contact::Tracked);
                if let Some(contact) = updated_contact {
                    self.contacts.insert(*id, contact)
                }
            }
            None => (),
        };
    }
    pub fn adjust<T: Acceleration>(&mut self, emitter: &T) {
        self.job_index = if (self.job_index + 1) >= self.job_rotation.len() {
            0
        } else {
            self.job_index + 1
        };
        match self.job_rotation.get(self.job_index) {
            Some(RadarJob::Search) => self.search.adjust(emitter),
            Some(RadarJob::Track(id)) => match self.contacts.get(*id) {
                Some(Contact::Tracked(contact)) => self.track.adjust(contact, emitter),
                Some(Contact::Scanned(contact)) => {
                    self.track.adjust(&TrackedContact::from(contact), emitter)
                }
                None => debug!("!!! => contact not found"),
            },
            None => (),
        }
    }
    pub fn set_job_rotation(&mut self, rotation: &[RadarJob]) {
        self.job_rotation = rotation.to_owned();
    }
    pub fn get_job_rotation(&mut self) -> &[RadarJob] {
        return &self.job_rotation;
    }
    pub fn draw_contacts(&self) {
        debug!("--------------------------------");
        debug!("Radar");
        debug!("Contacts:   {}", self.contacts.count());
        debug!("Job rota:   {:?}", self.job_rotation);
        debug!("Next job:   {:?}", self.job_index);
        self.contacts.draw_contacts();
        debug!("--------------------------------");
    }
}
}
pub mod search {
use oort_api::debug;
use oort_api::prelude::{current_time, TICK_LENGTH};
use super::{
    common::Radar,
    contacts::SearchContact,
    emitter::Emitter,
    math::kinematics::{Acceleration, Position},
};
#[derive(Clone, PartialEq, Debug)]
pub struct ScanningRadar {
    last_heading: f64,
    last_contact: Option<SearchContact>,
}
impl Radar for ScanningRadar {}
impl ScanningRadar {
    pub fn new() -> Self {
        return Self {
            last_heading: 0.0,
            last_contact: None,
        };
    }
}
impl ScanningRadar {
    const STANDARD_WIDTH: f64 = std::f64::consts::PI / 8.0;
    pub fn scan<T: Position>(&mut self, emitter: &T) -> Option<SearchContact> {
        let emitter = Emitter::new(emitter, self);
        let contact = self
            .get_scan()
            .map(|s| SearchContact::new(current_time(), &emitter, &s));
        self.last_heading = self.get_heading();
        self.last_contact = contact.clone();
        return contact;
    }
    pub fn adjust<T: Acceleration>(&self, emitter: &T) {
        self.set_width(Self::STANDARD_WIDTH);
        if let Some(contact) = &self.last_contact {
            let area = contact.get_area_after(contact.time_elapsed() + TICK_LENGTH);
            let furthest_point = area.max_distance_to(&emitter.position_after(TICK_LENGTH));
            self.set_heading(self.last_heading);
            self.set_min_distance(furthest_point);
            self.set_max_distance(Self::MAX_RADAR_RANGE);
        } else {
            let next_heading = self.last_heading + Self::STANDARD_WIDTH;
            self.set_heading(next_heading);
            self.set_min_distance(0.0);
            self.set_max_distance(Self::MAX_RADAR_RANGE);
        }
    }
}
}
pub mod track {
use oort_api::prelude::*;
use super::{
    common::Radar,
    contacts::{Contact, TrackedContact},
    emitter::Emitter,
    math::kinematics::{Acceleration, Position},
};
#[derive(Clone, PartialEq, Debug)]
pub struct TrackingRadar();
impl Radar for TrackingRadar {}
impl TrackingRadar {
    pub fn new() -> Self {
        return Self();
    }
}
impl TrackingRadar {
    const STANDARD_WIDTH: f64 = std::f64::consts::PI / 32.0;
    pub fn scan<T: Position>(&mut self, target: Contact, emitter: &T) -> Option<TrackedContact> {
        let mut target = match target {
            Contact::Scanned(contact) => TrackedContact::from(contact),
            Contact::Tracked(contact) => contact,
        };
        let emitter = Emitter::new(emitter, self);
        return self.get_scan().map(|s| {
            target.update(s, &emitter);
            target
        });
    }
    pub fn adjust<T: Acceleration>(&self, target: &TrackedContact, emitter: &T) {
        let time_elapsed = target.time_elapsed() + TICK_LENGTH;
        let area = target.get_area_after(time_elapsed);
        let (min, max) = area.minmax_distance_to(&emitter.position_after(TICK_LENGTH));
        debug!("centre: {}", area.position());
        debug!("range:  {} - {}", min, max);
        self.set_width(Self::STANDARD_WIDTH);
        self.set_heading(emitter.bearing_to(target));
        self.set_min_distance(min);
        self.set_max_distance(max);
    }
}
}
use super::{draw, math, ship};
pub use self::manager::{RadarJob, RadarManager};
}
pub mod radio {
pub mod message {
use std::mem::MaybeUninit;
use oort_api::prelude::*;
#[derive(Clone, PartialEq, Debug)]
pub enum RadioMessage {
    Position(Vec2),
    Unknown,
}
impl RadioMessage {
    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            Self::Position(position) => [&[1], &serialise_vec2(position)[..]].concat(),
            Self::Unknown => Vec::new(),
        }
    }
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let type_byte = bytes[0];
        match type_byte {
            1 => Self::Position(deserialise_vec2(bytes[1..17].try_into().unwrap())),
            _ => Self::Unknown,
        }
    }
}
fn serialise_vec2(vec: &Vec2) -> [u8; 16] {
    let mut bytes: [MaybeUninit<u8>; 16] = unsafe { MaybeUninit::uninit().assume_init() };
    bytes[..8].copy_from_slice(&vec.x.to_le_bytes().map(MaybeUninit::new));
    bytes[8..].copy_from_slice(&vec.y.to_le_bytes().map(MaybeUninit::new));
    return unsafe { std::mem::transmute(bytes) };
}
fn deserialise_vec2(bytes: [u8; 16]) -> Vec2 {
    let bytes: [[u8; 8]; 2] = unsafe { std::mem::transmute(bytes) };
    return vec2(f64::from_le_bytes(bytes[0]), f64::from_le_bytes(bytes[1]));
}
}
pub mod ship_radio {
use oort_api::prelude::*;
use super::message::RadioMessage;
#[derive(Clone, PartialEq, Debug)]
pub struct Radio {}
impl Radio {
    pub fn new() -> Self {
        return Self {};
    }
    pub fn set_channel(&self, channel: usize) {
        set_radio_channel(channel);
    }
    pub fn get_channel(&self) -> usize {
        return get_radio_channel();
    }
    pub fn send(&self, message: RadioMessage) {
        send_bytes(&message.to_bytes());
    }
    pub fn receive(&self) -> Option<RadioMessage> {
        return receive_bytes().map(|bytes| RadioMessage::from_bytes(&bytes));
    }
}
}
pub use message::RadioMessage;
pub use ship_radio::Radio;
}
pub mod scenario {
use oort_api::prelude::scenario_name;
pub enum Scenario {
    Sandbox,
    FighterDual,
    Unknown,
}
impl Scenario {
    pub fn current() -> Self {
        match scenario_name() {
            "sandbox" => Self::Sandbox,
            "fighter_dual" => Self::FighterDual,
            _ => Self::Unknown,
        }
    }
}
}
pub mod ship {
pub mod class {
use super::experimental::{ContactDrawer, RadarTester};
pub trait ShipClassLoop {
    fn tick(&mut self);
}
pub enum ShipClass {
    Fighter(Box<dyn ShipClassLoop>),
    Missile(Box<dyn ShipClassLoop>),
    ExContactDrawer(ContactDrawer),
    ExRadarTester(RadarTester),
    Unknown(),
}
}
pub mod experimental {
pub mod contact_draw {
use oort_api::prelude::*;
use super::{
    draw::{self, Colour},
    math::{
        geometry::Ellipse,
        kinematics::{Acceleration, AngularVelocity, Heading, Position, Velocity},
    },
};
pub struct ContactDrawer {
    acceleration: Vec2,
}
impl Position for ContactDrawer {
    fn position(&self) -> Vec2 {
        return position();
    }
}
impl Velocity for ContactDrawer {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}
impl Acceleration for ContactDrawer {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl Heading for ContactDrawer {
    fn heading(&self) -> f64 {
        return heading();
    }
}
impl AngularVelocity for ContactDrawer {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}
const BEARING_NOISE_FACTOR: f64 = 1e1 * (std::f64::consts::TAU / 360.0);
const DISTANCE_NOISE_FACTOR: f64 = 1e4;
impl ContactDrawer {
    pub fn new() -> Self {
        debug!("spawn fighter team 0 position (50, 0) heading 0");
        debug!("spawn missile team 1 position (3000, 3000) heading 0");
        return Self {
            acceleration: vec2(0.0, 0.0),
        };
    }
    pub fn tick(&mut self) {
        set_radar_heading(std::f64::consts::FRAC_PI_4);
        draw_line(
            vec2(0.0, 0.0),
            vec2(100.0, 0.0).rotate(std::f64::consts::FRAC_PI_4),
            Colour::Red as u32,
        );
        if let Some(scan) = scan() {
            let distance = self.distance_to(&scan.position);
            let bearing = scan.position.bearing_to(self);
            let mean = 0.0;
            let stddev = 1.0;
            let min_probability = 0.001;
            let max_rng = max_value(min_probability, mean, stddev);
            let error_factor = 10.0_f64.powf(-scan.snr / 10.0);
            let bearing_error = BEARING_NOISE_FACTOR * error_factor * max_rng;
            let distance_error = DISTANCE_NOISE_FACTOR * error_factor * max_rng;
            let mut distance_errors = Vec::new();
            let mut distance_rng = 0.0;
            let step = max_rng / 20.0;
            while distance_rng <= (max_rng + step) {
                let prob = probability_of(distance_rng, mean, stddev);
                let dist = distance_rng * DISTANCE_NOISE_FACTOR * error_factor;
                distance_errors.push((dist, prob));
                distance_rng += step;
            }
            let mut errors = Vec::new();
            for (dist, prob) in distance_errors {
                let bearing_prob = min_probability / prob;
                let bearing_rng = max_value(bearing_prob, mean, stddev);
                let bear = bearing_rng * BEARING_NOISE_FACTOR * error_factor;
                errors.push((dist, bear));
            }
            let mut points = Vec::new();
            for (dist_error, bear_error) in errors.iter() {
                let dist = distance + dist_error;
                let bear = bearing + bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }
            for (dist_error, bear_error) in errors.iter() {
                let dist = distance - dist_error;
                let bear = bearing + bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }
            for (dist_error, bear_error) in errors.iter() {
                let dist = distance - dist_error;
                let bear = bearing - bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }
            for (dist_error, bear_error) in errors.iter() {
                let dist = distance + dist_error;
                let bear = bearing - bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }
            debug!("points: {}", points.len());
            for points in points.windows(2) {
                let (p1, p2) = (points[0], points[1]);
                draw::line(&p1, &p2, Colour::Green);
            }
            let width = f64::atan(bearing_error) * distance * 2.0;
            let height = distance_error * 2.0;
            let ellipse = Ellipse::new(&scan.position, bearing, width, height);
            ellipse.draw(Colour::Purple);
            let ellipse =
                Ellipse::new(&self.position(), std::f64::consts::FRAC_PI_4, width, height);
            ellipse.draw(Colour::Purple);
        }
    }
}
fn probability_of(value: f64, mean: f64, stddev: f64) -> f64 {
    let num = f64::exp(-0.5 * f64::powf((value - mean) / stddev, 2.0));
    let den = stddev * f64::sqrt(2.0 * std::f64::consts::PI);
    return num / den;
}
fn max_value(prob: f64, mean: f64, stddev: f64) -> f64 {
    return (stddev
        * f64::sqrt(-2.0 * f64::ln(prob * stddev * f64::sqrt(2.0 * std::f64::consts::PI))))
        + mean;
}
}
pub mod radar_test {
use oort_api::prelude::*;
use super::{
    control::{Rotation, Translation},
    draw::{self, Colour},
    math::kinematics::{
        Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity,
    },
    radar::{contacts::Contact, RadarJob, RadarManager},
};
pub struct RadarTester {
    radar: RadarManager,
    acceleration: Vec2,
    target: Option<usize>,
}
impl Position for RadarTester {
    fn position(&self) -> Vec2 {
        return position();
    }
}
impl Velocity for RadarTester {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}
impl Acceleration for RadarTester {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl Heading for RadarTester {
    fn heading(&self) -> f64 {
        return heading();
    }
}
impl AngularVelocity for RadarTester {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}
impl Translation for RadarTester {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}
impl Rotation for RadarTester {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}
impl RadarTester {
    const BULLET_SPEED: f64 = 1000.0; // m/s
    pub fn new() -> Self {
        debug!("spawn fighter team 0 position (50, 0) heading 0");
        debug!("spawn missile team 1 position (3000, 3000) heading 0");
        let mut radar = RadarManager::new();
        radar.set_job_rotation(&[RadarJob::Search]);
        return Self {
            radar,
            acceleration: vec2(0.0, 0.0),
            target: None,
        };
    }
    pub fn tick(&mut self) {
        self.radar.scan(&self.position());
        debug!("Target: {:?}", self.target);
        let current_target = self
            .target
            .and_then(|id| Some(id).zip(self.radar.contacts.get(id)));
        if let Some((target_id, target)) = current_target {
            let other_contacts = self
                .radar
                .contacts
                .iter()
                .filter(|(_, c)| matches!(c, Contact::Scanned(_)));
            let missiles = other_contacts.filter(|(_, c)| c.is_class(Class::Missile));
            let priority = missiles.min_by_key(|(_, c)| c.distance_to(self) as u32);
            let priority = priority.filter(|(&id, _)| id != target_id);
            let priority = priority.filter(|(_, m)| m.distance_to(self) < target.distance_to(self));
            if let Some((priority_id, _)) = priority {
                self.target = Some(*priority_id);
                let jobs = [RadarJob::Track(*priority_id), RadarJob::Search];
                self.radar.contacts.track(*priority_id);
                self.radar.set_job_rotation(&jobs);
                None
            } else {
                Some(target)
            }
        } else {
            let new_target = self
                .radar
                .contacts
                .iter()
                .min_by_key(|(_, c)| c.distance_to(self) as u32);
            if let Some((id, _)) = new_target {
                self.target = Some(*id);
                let jobs = [RadarJob::Track(*id), RadarJob::Search];
                self.radar.contacts.track(*id);
                self.radar.set_job_rotation(&jobs);
            } else {
                self.target = None;
                let jobs = [RadarJob::Search];
                self.radar.set_job_rotation(&jobs);
            }
            None
        };
        self.radar.adjust(&KinematicModel::from(&*self));
        self.radar.draw_contacts();
    }
}
}
use super::{control, draw, math, radar};
pub use self::{contact_draw::ContactDrawer, radar_test::RadarTester};
}
pub mod fighter {
pub mod default {
use oort_api::prelude::*;
use super::{
    class::ShipClassLoop,
    control::{Rotation, Translation},
    draw::{self, Colour, Trail},
    math::{
        kinematics::{Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity},
        FiringSolution,
    },
    radar::{contacts::Contact, RadarJob, RadarManager},
    radio::{Radio, RadioMessage},
};
pub struct DefaultFighter {
    radar: RadarManager,
    radio: Radio,
    acceleration: Vec2,
    target: Option<usize>,
    target_trail_actual: Trail,
    target_trail_aim: Trail,
}
impl Position for DefaultFighter {
    fn position(&self) -> Vec2 {
        return position();
    }
}
impl Velocity for DefaultFighter {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}
impl Acceleration for DefaultFighter {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl Heading for DefaultFighter {
    fn heading(&self) -> f64 {
        return heading();
    }
}
impl AngularVelocity for DefaultFighter {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}
impl Translation for DefaultFighter {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}
impl Rotation for DefaultFighter {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}
impl DefaultFighter {
    const BULLET_SPEED: f64 = 1000.0; // m/s
    pub fn new() -> Self {
        let mut radar = RadarManager::new();
        radar.set_job_rotation(&[RadarJob::Search]);
        return Self {
            radar,
            radio: Radio::new(),
            acceleration: vec2(0.0, 0.0),
            target: None,
            target_trail_actual: Trail::with_length(256),
            target_trail_aim: Trail::with_length(256),
        };
    }
}
impl DefaultFighter {
    fn fire_guns(&self) {
        fire(0);
    }
    fn launch_missile(&mut self) {
        fire(1);
        self.radio.set_channel(0);
        if let Some((_, target)) = self
            .radar
            .contacts
            .iter()
            .find(|(_, c)| matches!(c, Contact::Tracked(_)))
        {
            self.radio.send(RadioMessage::Position(target.position()));
        }
    }
}
impl ShipClassLoop for DefaultFighter {
    fn tick(&mut self) {
        self.radar.scan(&self.position());
        let current_target = self
            .target
            .and_then(|id| Some(id).zip(self.radar.contacts.get(id)));
        let current_target = if let Some((target_id, target)) = current_target {
            let other_contacts = self
                .radar
                .contacts
                .iter()
                .filter(|(_, c)| matches!(c, Contact::Scanned(_)));
            let missiles = other_contacts.filter(|(_, c)| c.is_class(Class::Missile));
            let priority = missiles.min_by_key(|(_, c)| c.distance_to(self) as u32);
            let priority = priority.filter(|(&id, _)| id != target_id);
            let priority = priority.filter(|(_, m)| m.distance_to(self) < target.distance_to(self));
            if let Some((priority_id, _)) = priority {
                self.target = Some(*priority_id);
                let jobs = [RadarJob::Track(*priority_id), RadarJob::Search];
                self.radar.set_job_rotation(&jobs);
                None
            } else {
                Some(target)
            }
        } else {
            let new_target = self
                .radar
                .contacts
                .iter()
                .min_by_key(|(_, c)| c.distance_to(self) as u32);
            if let Some((id, _)) = new_target {
                self.target = Some(*id);
                let jobs = [RadarJob::Track(*id), RadarJob::Search];
                self.radar.set_job_rotation(&jobs);
            }
            None
        };
        let firing_solution = if let Some(Contact::Tracked(contact)) = current_target {
            self.target_trail_actual.update(contact);
            debug!("Target velocity: {}", contact.velocity());
            debug!("Target accel: {}", contact.acceleration());
            FiringSolution::new(self, Self::BULLET_SPEED, contact)
        } else {
            let map_centre = vec2(0.0, 0.0);
            self.turn_to_face(&map_centre);
            self.accelerate_towards(&map_centre);
            None
        };
        if let Some(solution) = firing_solution {
            debug!("Engaging target");
            self.turn_to_track(&solution);
            self.accelerate_towards(&solution);
            if self.relative_bearing_to(&solution).abs() < (PI / 4.0) {
                self.launch_missile();
            }
            if self.relative_bearing_to(&solution).abs() < 0.02 {
                self.fire_guns();
            }
            self.target_trail_aim.update(&solution);
            draw::aim_reticle(&solution);
        }
        self.radar.adjust(&KinematicModel::from(&*self));
        self.radar.draw_contacts();
    }
}
}
pub mod dualist {
use oort_api::prelude::*;
use super::{
    class::ShipClassLoop,
    control::{Rotation, Translation},
    draw::{self, Colour, Trail},
    math::{
        kinematics::{Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity},
        FiringSolution,
    },
    radar::{contacts::Contact, RadarJob, RadarManager},
    radio::{Radio, RadioMessage},
};
pub struct Dualist {
    radar: RadarManager,
    radio: Radio,
    acceleration: Vec2,
    target: Option<usize>,
    target_trail_actual: Trail,
    target_trail_aim: Trail,
}
impl Position for Dualist {
    fn position(&self) -> Vec2 {
        return position();
    }
}
impl Velocity for Dualist {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}
impl Acceleration for Dualist {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl Heading for Dualist {
    fn heading(&self) -> f64 {
        return heading();
    }
}
impl AngularVelocity for Dualist {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}
impl Translation for Dualist {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}
impl Rotation for Dualist {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}
impl Dualist {
    const BULLET_SPEED: f64 = 1000.0; // m/s
    pub fn new() -> Self {
        let mut radar = RadarManager::new();
        radar.set_job_rotation(&[RadarJob::Search]);
        return Self {
            radar,
            radio: Radio::new(),
            acceleration: vec2(0.0, 0.0),
            target: None,
            target_trail_actual: Trail::with_length(256),
            target_trail_aim: Trail::with_length(256),
        };
    }
}
impl Dualist {
    fn fire_guns(&self) {
        fire(0);
    }
    fn launch_missile(&mut self) {
        fire(1);
        self.radio.set_channel(0);
        if let Some((_, target)) = self
            .radar
            .contacts
            .iter()
            .find(|(_, c)| matches!(c, Contact::Tracked(_)))
        {
            self.radio.send(RadioMessage::Position(target.position()));
        }
    }
}
impl ShipClassLoop for Dualist {
    fn tick(&mut self) {
        self.radar.scan(&self.position());
        let current_target = self
            .target
            .and_then(|id| Some(id).zip(self.radar.contacts.get(id)));
        let current_target = if let Some((target_id, target)) = current_target {
            let other_contacts = self
                .radar
                .contacts
                .iter()
                .filter(|(_, c)| matches!(c, Contact::Scanned(_)));
            let missiles = other_contacts.filter(|(_, c)| c.is_class(Class::Missile));
            let priority = missiles.min_by_key(|(_, c)| c.distance_to(self) as u32);
            let priority = priority.filter(|(&id, _)| id != target_id);
            let priority = priority.filter(|(_, m)| m.distance_to(self) < target.distance_to(self));
            if let Some((priority_id, _)) = priority {
                self.target = Some(*priority_id);
                let jobs = [RadarJob::Track(*priority_id), RadarJob::Search];
                self.radar.set_job_rotation(&jobs);
                None
            } else {
                Some(target)
            }
        } else {
            let new_target = self
                .radar
                .contacts
                .iter()
                .min_by_key(|(_, c)| c.distance_to(self) as u32);
            if let Some((id, _)) = new_target {
                self.target = Some(*id);
                let jobs = [RadarJob::Track(*id), RadarJob::Search];
                self.radar.set_job_rotation(&jobs);
            }
            None
        };
        let firing_solution = if let Some(Contact::Tracked(contact)) = current_target {
            self.target_trail_actual.update(contact);
            debug!("Target velocity: {}", contact.velocity());
            debug!("Target accel: {}", contact.acceleration());
            FiringSolution::new(self, Self::BULLET_SPEED, contact)
        } else {
            let map_centre = vec2(0.0, 0.0);
            self.turn_to_face(&map_centre);
            self.accelerate_towards(&map_centre);
            None
        };
        if let Some(solution) = firing_solution {
            debug!("Engaging target");
            self.turn_to_track(&solution);
            self.accelerate_towards(&solution);
            if self.relative_bearing_to(&solution).abs() < (PI / 4.0) {
                self.launch_missile();
            }
            if self.relative_bearing_to(&solution).abs() < 0.02 {
                self.fire_guns();
            }
            self.target_trail_aim.update(&solution);
            draw::aim_reticle(&solution);
        }
        self.radar.adjust(&KinematicModel::from(&*self));
        self.radar.draw_contacts();
    }
}
}
use super::{class, control, draw, math, radar, radio};
pub use self::{default::DefaultFighter, dualist::Dualist};
}
pub mod missile {
pub mod default {
use oort_api::prelude::*;
use super::super::super::ship::class::ShipClassLoop;
use super::{
    control::{Rotation, Translation},
    draw::{self, Colour},
    math::{
        kinematics::{Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity},
        Intercept,
    },
    radar::{contacts::Contact, RadarJob, RadarManager},
    radio::{Radio, RadioMessage},
    stats::MaxAcceleration,
};
pub struct DefaultMissile {
    radar: RadarManager,
    radio: Radio,
    target_position: Option<Vec2>,
    acceleration: Vec2,
}
impl Position for DefaultMissile {
    fn position(&self) -> Vec2 {
        return position();
    }
}
impl Velocity for DefaultMissile {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}
impl Acceleration for DefaultMissile {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}
impl Heading for DefaultMissile {
    fn heading(&self) -> f64 {
        return heading();
    }
}
impl AngularVelocity for DefaultMissile {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}
impl Translation for DefaultMissile {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        let angle = angle_diff(self.heading(), acceleration.angle());
        let forward = acceleration.length() * f64::cosh(angle);
        let lateral = acceleration.length() * f64::sinh(angle);
        let forward = forward.clamp(max_backward_acceleration(), max_forward_acceleration());
        let lateral = lateral.clamp(-max_lateral_acceleration(), max_lateral_acceleration());
        let accel = vec2(forward, lateral).rotate(acceleration.angle());
        accelerate(accel);
        self.acceleration = accel;
    }
}
impl Rotation for DefaultMissile {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}
impl DefaultMissile {
    pub fn new() -> Self {
        let mut radar = RadarManager::new();
        radar.set_job_rotation(&[RadarJob::Search, RadarJob::Track(0)]);
        let radio = Radio::new();
        let target_position = if let Some(RadioMessage::Position(position)) = radio.receive() {
            let heading = position.angle();
            Some(position)
        } else {
            None
        };
        return Self {
            radar,
            radio,
            target_position,
            acceleration: vec2(0.0, 0.0),
        };
    }
}
impl ShipClassLoop for DefaultMissile {
    fn tick(&mut self) {
        self.radar.scan(&self.position());
        let target = if let Some((_, contact)) = self
            .radar
            .contacts
            .iter()
            .find(|(_, c)| matches!(c, Contact::Tracked(_)))
        {
            Some(contact).cloned()
        } else {
            let closest_contact = self
                .radar
                .contacts
                .iter()
                .filter(|(_, c)| matches!(c, Contact::Scanned(_)))
                .min_by(|(_, c1), (_, c2)| {
                    c1.distance_to(self)
                        .partial_cmp(&c2.distance_to(self))
                        .unwrap()
                });
            if let Some((&id, _)) = closest_contact {
                let jobs = [RadarJob::Track(id)];
                self.radar.set_job_rotation(&jobs);
            }
            None
        };
        if let Some(Contact::Tracked(target)) = target {
            let acceleration = if active_abilities().get_ability(Ability::Boost) {
                MaxAcceleration::from(Class::Missile).forward + 100.0
            } else {
                MaxAcceleration::from(Class::Missile).forward
            };
            let solution = Intercept::new(self, acceleration, &target);
            self.turn_to_face(&solution);
            self.accelerate_towards(&solution);
            draw::aim_reticle(&solution);
            if target.distance_to(self) < 180.0 {
                explode();
            } else if target.distance_to(self) < 400.0 {
                self.turn_to_track(&target);
                draw::aim_reticle(&target);
            } else if self.relative_bearing_to(&solution) < 0.3 {
                activate_ability(Ability::Boost);
            }
        }
        self.radar.adjust(&KinematicModel::from(&*self));
        draw::heading(self);
    }
}
}
use super::{control, draw, math, radar, radio, stats};
pub use self::default::DefaultMissile;
}
pub mod stats {
pub mod acceleration {
use oort_api::prelude::*;
#[derive(Clone, PartialEq, PartialOrd, Debug)]
pub struct MaxAcceleration {
    pub forward: f64,
    pub reverse: f64,
    pub lateral: f64,
    pub angular: f64,
}
impl From<Class> for MaxAcceleration {
    fn from(value: Class) -> Self {
        return match value {
            Class::Fighter => Self {
                forward: 60.0,
                reverse: 30.0,
                lateral: 30.0,
                angular: 2.0 * std::f64::consts::PI,
            },
            Class::Frigate => Self {
                forward: 10.0,
                reverse: 5.0,
                lateral: 5.0,
                angular: std::f64::consts::PI / 4.0,
            },
            Class::Cruiser => Self {
                forward: 5.0,
                reverse: 2.5,
                lateral: 2.5,
                angular: std::f64::consts::PI / 8.0,
            },
            Class::Asteroid => Self {
                forward: 0.0,
                reverse: 0.0,
                lateral: 0.0,
                angular: 0.0,
            },
            Class::Target => todo!(),
            Class::Missile => Self {
                forward: 300.0,
                reverse: 0.0,
                lateral: 100.0,
                angular: 4.0 * std::f64::consts::PI,
            },
            Class::Torpedo => Self {
                forward: 70.0,
                reverse: 0.0,
                lateral: 20.0,
                angular: 2.0 * std::f64::consts::PI,
            },
            Class::Unknown => panic!(),
        };
    }
}
impl MaxAcceleration {
    pub fn magnitude(&self) -> f64 {
        let medial = f64::max(self.forward, self.reverse);
        return f64::sqrt(medial.powf(2.0) + self.lateral.powf(2.0));
    }
}
}
pub mod dimensions {
use oort_api::prelude::*;
#[derive(Clone, PartialEq, PartialOrd, Debug)]
pub struct Dimensions {
    pub fore: f64,
    pub aft: f64,
    pub left: f64,
    pub right: f64,
}
impl From<Class> for Dimensions {
    fn from(value: Class) -> Self {
        return match value {
            Class::Fighter => Self {
                fore: 12.0,
                aft: 6.0,
                left: 8.0,
                right: 8.0,
            },
            Class::Frigate => todo!(),
            Class::Cruiser => todo!(),
            Class::Asteroid => todo!(),
            Class::Target => todo!(),
            Class::Missile => Self {
                fore: 5.0,
                aft: 1.0,
                left: 3.0,
                right: 3.0,
            },
            Class::Torpedo => todo!(),
            Class::Unknown => todo!(),
        };
    }
}
impl Dimensions {
    pub fn length(&self) -> f64 {
        return self.fore + self.aft;
    }
    pub fn width(&self) -> f64 {
        return self.left + self.right;
    }
    pub fn longest(&self) -> f64 {
        return f64::max(
            f64::max(self.fore, self.aft),
            f64::max(self.left, self.right),
        );
    }
}
}
pub use acceleration::MaxAcceleration;
pub use dimensions::Dimensions;
}
use super::{control, draw, math, radar, radio};
pub use class::ShipClass;
}
use self::{
    scenario::Scenario,
    ship::{
        experimental::{ContactDrawer, RadarTester},
        fighter::{DefaultFighter, Dualist},
        missile::DefaultMissile,
        ShipClass,
    },
};
pub struct Ship {
    class: ShipClass,
}
impl Default for Ship {
    fn default() -> Self {
        return Self::from(ShipClass::Unknown());
    }
}
impl From<ShipClass> for Ship {
    fn from(class: ShipClass) -> Self {
        return Self { class };
    }
}
impl Ship {
    const SANDBOX_MODE: &str = "radar_test";
    pub fn new() -> Ship {
        let scenario = Scenario::current();
        if matches!(scenario, Scenario::Sandbox) {
            return match Self::SANDBOX_MODE {
                "radar_test" => Self::from(ShipClass::ExRadarTester(RadarTester::new())),
                "contact_draw" => Self::from(ShipClass::ExContactDrawer(ContactDrawer::new())),
                _ => panic!("Error - Unknown sandbox mode"),
            };
        }
        use ShipClass::*;
        return match class() {
            Class::Fighter => match Scenario::current() {
                Scenario::FighterDual => Self::from(Fighter(Box::new(Dualist::new()))),
                _ => Self::from(Fighter(Box::new(DefaultFighter::new()))),
            },
            Class::Missile => Self::from(Missile(Box::new(DefaultMissile::new()))),
            _ => Self::default(),
        };
    }
    pub fn tick(&mut self) {
        match &mut self.class {
            ShipClass::Fighter(fighter) => fighter.tick(),
            ShipClass::Missile(missile) => missile.tick(),
            ShipClass::ExContactDrawer(contact_drawer) => contact_drawer.tick(),
            ShipClass::ExRadarTester(radar_tester) => radar_tester.tick(),
            ShipClass::Unknown() => (),
        }
    }
}
}
pub use oort_ai::Ship;
