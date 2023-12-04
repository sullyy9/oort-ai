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

////////////////////////////////////////////////////////////////

// Make each field a closure?
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

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

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
        // https://en.wikipedia.org/wiki/Angular_velocity
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

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

pub trait Heading: Position {
    fn heading(&self) -> f64;

    fn relative_bearing_to<T: Position>(&self, target: &T) -> f64 {
        let target_vector = target.position() - self.position();
        return angle_diff(self.heading(), target_vector.angle());
    }
}

////////////////////////////////////////////////////////////////

pub trait AngularVelocity: Heading {
    fn angular_velocity(&self) -> f64;

    fn heading_after(&self, seconds: f64) -> f64 {
        let heading = self.heading() + PI; // Normalise between 0 and 2 * PI.
        let heading = heading + (self.angular_velocity() * seconds);

        let heading = heading % (2.0 * PI); // Wrap.
        return heading - PI;
    }
}

////////////////////////////////////////////////////////////////

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
