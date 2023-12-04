use oort_api::prelude::*;

use super::{
    kinematics::{Acceleration, Position, Velocity},
    polynomial::{self, Roots},
};

/// TODO
/// ----
/// Actually calculate the velocity of the impact point.
/// relative velocity of target at impact time?
pub struct FiringSolution {
    impact_time: f64,
    impact_point: Vec2,
    target_velocity: Vec2,
    target_acceleration: Vec2,
}

////////////////////////////////////////////////////////////////

impl Position for FiringSolution {
    fn position(&self) -> Vec2 {
        return self.impact_point;
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for FiringSolution {
    fn velocity(&self) -> Vec2 {
        return self.target_velocity;
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for FiringSolution {
    fn acceleration(&self) -> Vec2 {
        return self.target_acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl FiringSolution {
    pub fn new<T: Velocity, U: Acceleration>(
        shooter: &T,
        projectile_speed: f64,
        target: &U,
    ) -> Option<Self> {
        // Use relative velocity to account for own movement.
        let tarpos = target.position_relative_to(shooter);
        let tarvel = target.velocity_relative_to(shooter);
        let taracc = target.acceleration();

        let a = taracc.dot(taracc) / 4.0;
        let b = taracc.dot(tarvel);
        let c = tarvel.dot(tarvel) + taracc.dot(tarpos) - projectile_speed.powf(2.0);
        let d = 2.0 * tarvel.dot(tarpos);
        let e = tarpos.dot(tarpos);

        let roots = polynomial::find_roots_quartic(a, b, c, d, e);

        // We always expect there to be four roots.
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
