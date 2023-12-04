use oort_api::prelude::*;

use super::{
    draw::{self, Colour},
    kinematics::Position,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct Circle {
    pub centre: Vec2,
    pub radius: f64,
}

////////////////////////////////////////////////////////////////

impl Circle {
    pub fn new<T: Position>(centre: &T, radius: f64) -> Self {
        return Self {
            centre: centre.position(),
            radius,
        };
    }
}

////////////////////////////////////////////////////////////////

impl Circle {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        return point.position().distance_to(&self.centre) < self.radius;
    }
}

////////////////////////////////////////////////////////////////

impl Circle {
    pub fn draw(&self, colour: Colour) {
        draw::regular_polygon(&self.centre, self.radius, 8, 0.0, colour)
    }
}

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////
