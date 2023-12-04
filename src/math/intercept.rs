use oort_api::prelude::*;

use super::{
    kinematics::{Acceleration, Position},
    polynomial::{self, Roots},
};

pub struct Intercept {
    time: f64,
    point: Vec2,
}

////////////////////////////////////////////////////////////////

impl Position for Intercept {
    fn position(&self) -> Vec2 {
        return self.point;
    }
}

////////////////////////////////////////////////////////////////

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

        // There will either be 2 or four roots.
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
