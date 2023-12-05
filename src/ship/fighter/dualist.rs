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

////////////////////////////////////////////////////////////////

pub struct Dualist {
    radar: RadarManager,
    radio: Radio,

    acceleration: Vec2,

    target: Option<usize>,

    target_trail_actual: Trail,
    target_trail_aim: Trail,
}

////////////////////////////////////////////////////////////////

impl Position for Dualist {
    fn position(&self) -> Vec2 {
        return position();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for Dualist {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for Dualist {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Heading for Dualist {
    fn heading(&self) -> f64 {
        return heading();
    }
}

////////////////////////////////////////////////////////////////

impl AngularVelocity for Dualist {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Translation for Dualist {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Rotation for Dualist {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

impl ShipClassLoop for Dualist {
    fn tick(&mut self) {
        // Update radar contacts.
        self.radar.scan(&self.position());

        // Find the current target.
        let current_target = self
            .target
            .and_then(|id| Some(id).zip(self.radar.contacts.get(id)));

        // Check that their's not a higher priority target.
        let current_target = if let Some((target_id, target)) = current_target {
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

                let jobs = [RadarJob::Track(*priority_id), RadarJob::Search];
                self.radar.set_job_rotation(&jobs);

                None
            } else {
                Some(target)
            }
        } else {
            // If the previous target has been lost start tracking a new one.
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

        // If we have a tracked target, get a firing solution.
        let firing_solution = if let Some(Contact::Tracked(contact)) = current_target {
            self.target_trail_actual.update(contact);
            // self.target_trail_actual.draw(Colour::Green);

            debug!("Target velocity: {}", contact.velocity());
            debug!("Target accel: {}", contact.acceleration());

            FiringSolution::new(self, Self::BULLET_SPEED, contact)
        } else {
            let map_centre = vec2(0.0, 0.0);
            self.turn_to_face(&map_centre);
            self.accelerate_towards(&map_centre);

            None
        };

        // Engage the target using the firing solution.
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
            // self.target_trail_aim.draw(Colour::Yellow);
            draw::aim_reticle(&solution);
        }

        self.radar.adjust(&KinematicModel::from(&*self));
        // draw::heading(self);
        self.radar.draw_contacts();
    }
}

////////////////////////////////////////////////////////////////
