use oort_api::prelude::*;

use super::{
    draw::{self, Colour},
    kinematics::Position,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct Line {
    pub points: (Vec2, Vec2),
}

////////////////////////////////////////////////////////////////

impl Line {
    pub fn new<T: Position>(point1: &T, point2: &T) -> Self {
        return Self {
            points: (point1.position(), point2.position()),
        };
    }
}

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

impl Line {
    pub fn draw(&self, colour: Colour) {
        draw::line(&self.points.0, &self.points.1, colour);
    }
}

////////////////////////////////////////////////////////////////
