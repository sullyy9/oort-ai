use oort_api::prelude::*;

use super::math::kinematics::{Acceleration, AngularVelocity, Position, Velocity};

////////////////////////////////////////////////////////////////

pub trait Translation: Position {
    fn set_acceleration(&mut self, acceleration: Vec2);

    fn accelerate_towards<T: Position>(&self, target: &T) {
        let target_vector = target.position() - self.position();
        accelerate(target_vector);
    }
}

////////////////////////////////////////////////////////////////

pub trait Rotation: AngularVelocity + Velocity + Sized {
    fn set_angular_acceleration(&mut self, angular_acceleration: f64);

    // Create implementation for heading, position, vessel and tracked vessel?
    // If target is moving, we don't want final velocity to be 0.
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

        // Find the equation giving target bearing.
        // Find time to intercept.

        // Given a maximum deceleration, find the maximum velocity we could be at in our current
        // position to not overshoot.
        // v^2 = u^2 + 2as where a = 2.0 or -2.0 depending on whether bearing is + or -.
        // sqrt(v^2 - 2as)
        let decel = if target_bearing < 0.0 { 2.0 } else { -2.0 };
        // let decel = decel - target_acceleration;
        let max_velocity = f64::sqrt(
            (target.orbital_velocity_to(self).powf(2.0) - (2.0 * decel * target_bearing)).abs(),
        );

        let max_velocity = if target_bearing.is_sign_negative() {
            -max_velocity
        } else {
            max_velocity
        };

        // v = u + at
        // a = (v - u) / t
        // let acceleration = (velocity - self.angular_velocity()) / TICK_LENGTH;
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
