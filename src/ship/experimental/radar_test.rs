use oort_api::prelude::*;

use super::{
    control::{Rotation, Translation},
    draw::{self, Colour},
    math::kinematics::{
        Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity,
    },
    radar::{
        board::{ContactBoard, UniqueContactBoard},
        contacts::Contact,
        RadarManager,
    },
};

type Radar = RadarManager<UniqueContactBoard>;

////////////////////////////////////////////////////////////////

pub struct RadarTester {
    radar: Radar,

    acceleration: Vec2,

    target: Option<usize>,
}

////////////////////////////////////////////////////////////////

impl Position for RadarTester {
    fn position(&self) -> Vec2 {
        return position();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for RadarTester {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for RadarTester {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Heading for RadarTester {
    fn heading(&self) -> f64 {
        return heading();
    }
}

////////////////////////////////////////////////////////////////

impl AngularVelocity for RadarTester {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Translation for RadarTester {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Rotation for RadarTester {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}

////////////////////////////////////////////////////////////////

impl RadarTester {
    const BULLET_SPEED: f64 = 1000.0; // m/s

    pub fn new() -> Self {
        debug!("spawn fighter team 0 position (50, 0) heading 0");
        debug!("spawn missile team 1 position (3000, 3000) heading 0");

        return Self {
            radar: Radar::new(UniqueContactBoard::new()),
            acceleration: vec2(0.0, 0.0),
            target: None,
        };
    }

    pub fn tick(&mut self) {
        // Update radar contacts.
        self.radar.scan(&self.position());

        // Find the current target.
        debug!("Target: {:?}", self.target);
        let current_target = self
            .target
            .and_then(|id| Some(id).zip(self.radar.contacts.get(id)));

        // Check that their's not a higher priority target.
        if let Some((target_id, target)) = current_target {
            let other_contacts = self
                .radar
                .contacts
                .iter()
                .filter(|(_, c)| matches!(c, Contact::Search(_)));

            let missiles = other_contacts.filter(|(_, c)| c.class() == Class::Missile);

            let priority = missiles.min_by_key(|(_, c)| c.distance_to(self) as u32);
            let priority = priority.filter(|(&id, _)| id != target_id);
            let priority = priority.filter(|(_, m)| m.distance_to(self) < target.distance_to(self));

            if let Some((priority_id, _)) = priority {
                self.target = Some(*priority_id);

                if let Err(error) = self.radar.start_tracking(*priority_id) {
                    debug!("ERROR - {error:?}")
                }

                None
            } else {
                Some(target)
            }
        } else {
            if let Some(id) = self.target {
                self.radar.stop_tracking(id);
            }

            self.target = None;

            // If the previous target has been lost start tracking a new one.
            let new_target = self
                .radar
                .contacts
                .iter()
                .min_by_key(|(_, c)| c.distance_to(self) as u32);

            if let Some((id, _)) = new_target {
                self.target = Some(*id);

                if let Err(error) = self.radar.start_tracking(*id) {
                    debug!("ERROR - {error:?}")
                }
            }

            None
        };

        self.radar.adjust(&KinematicModel::from(&*self));
        self.radar.draw_contacts();
    }
}

////////////////////////////////////////////////////////////////
