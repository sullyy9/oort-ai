use oort_api::prelude::*;

use super::{
    class::ShipClassLoop,
    control::{Rotation, Translation},
    draw,
    math::{
        kinematics::{Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity},
        FiringSolution,
    },
    radar::{
        board::{ContactBoard, UniqueContactBoard},
        contacts::Contact,
        RadarManager,
    },
    radio::{Radio, RadioMessage},
};

type Radar = RadarManager<UniqueContactBoard>;

////////////////////////////////////////////////////////////////

pub struct Duelist {
    radar: Radar,
    radio: Radio,

    acceleration: Vec2,

    enemy_fighter: Option<usize>,
    enemy_missile: Option<usize>,
}

////////////////////////////////////////////////////////////////

impl Position for Duelist {
    fn position(&self) -> Vec2 {
        return position();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for Duelist {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for Duelist {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Heading for Duelist {
    fn heading(&self) -> f64 {
        return heading();
    }
}

////////////////////////////////////////////////////////////////

impl AngularVelocity for Duelist {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Translation for Duelist {
    fn set_acceleration(&mut self, acceleration: Vec2) {
        accelerate(acceleration);
        self.acceleration = acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Rotation for Duelist {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}

////////////////////////////////////////////////////////////////

impl Duelist {
    const BULLET_SPEED: f64 = 1000.0; // m/s

    pub fn new() -> Self {
        return Self {
            radar: Radar::new(UniqueContactBoard::new()),
            radio: Radio::new(),

            acceleration: vec2(0.0, 0.0),

            enemy_fighter: None,
            enemy_missile: None,
        };
    }
}

////////////////////////////////////////////////////////////////

impl Duelist {
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

impl ShipClassLoop for Duelist {
    fn tick(&mut self) {
        debug!("Role: Duelist");

        // Update radar contacts.
        self.radar.scan(&self.position());

        let get_contact_and_id = |id| self.radar.contacts.get(id).map(|c| (id, c));
        let get_contact = |id| self.radar.contacts.get(id);

        // Find the current target.
        let enemy_fighter = self.enemy_fighter.or_else(|| self.get_enemy_fighter());

        // Find an incoming missile.
        let enemy_missile = self
            .enemy_missile
            .or_else(|| self.get_closest_enemy_missile());

        // Configure the radar job rotation.
        let mut start_tracking = Vec::new();
        let mut stop_tracking = Vec::new();

        if let Some((id, Contact::Search(_))) = enemy_fighter.and_then(get_contact_and_id) {
            debug!("new enemy fighter");

            if let Some(old_id) = self.enemy_fighter {
                if old_id != id {
                    stop_tracking.push(old_id);
                }
            }

            start_tracking.push(id);
        }

        if enemy_fighter.is_none() {
            if let Some(id) = self.enemy_fighter {
                stop_tracking.push(id);
            }
        }

        // Manage the job for tracking an incoming missile.
        if let Some((id, Contact::Search(_))) = enemy_missile.and_then(get_contact_and_id) {
            debug!("new enemy missile");

            if let Some(old_id) = self.enemy_missile {
                if old_id != id {
                    stop_tracking.push(old_id);
                }
            }
            start_tracking.push(id);
        }

        if enemy_missile.is_none() {
            if let Some(old_id) = self.enemy_missile {
                stop_tracking.push(old_id);
            }
        }

        // Enagage a target. Prioritise missiles.
        let target = if let Some(Contact::Tracked(contact)) = enemy_missile.and_then(get_contact) {
            Some(contact)
        } else if let Some(Contact::Tracked(contact)) = enemy_fighter.and_then(get_contact) {
            Some(contact)
        } else {
            None
        };

        let firing_solution = target.and_then(|c| FiringSolution::new(self, Self::BULLET_SPEED, c));

        // Decide where to move.
        if let Some(fighter) = enemy_fighter.and_then(get_contact) {
            let pos = fighter.position();
            self.turn_to_face(&pos);
            self.accelerate_towards(&pos);
        } else {
            let map_centre = vec2(0.0, 0.0);
            self.turn_to_face(&map_centre);
            self.accelerate_towards(&map_centre);
        }

        // Save the targets.
        self.enemy_fighter = enemy_fighter;
        self.enemy_missile = enemy_missile;

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

            draw::aim_reticle(&solution);
        }

        // Start or stop tracking targets.
        for id in stop_tracking {
            self.radar.stop_tracking(id);
        }

        for id in start_tracking {
            if let Err(error) = self.radar.start_tracking(id) {
                debug!("ERROR - {:?}", error);
            }
        }

        self.radar.adjust(&KinematicModel::from(&*self));
        // draw::heading(self);
        self.radar.draw_contacts();
    }
}

////////////////////////////////////////////////////////////////

impl Duelist {
    /// Description
    /// -----------
    /// Get the id and contact of an enemy fighter from the contact board.
    /// There should only be 1 enemy fighter in this scenario.
    ///
    fn get_enemy_fighter(&self) -> Option<usize> {
        return self
            .radar
            .contacts
            .iter()
            .find(|(_, c)| c.class() == Class::Fighter)
            .map(|(id, _)| *id);
    }

    /// Description
    /// -----------
    /// Get the id and contact of the closest incoming missile from the contact board.
    ///
    fn get_closest_enemy_missile(&self) -> Option<usize> {
        return self
            .radar
            .contacts
            .iter()
            .filter(|(_, c)| c.class() == Class::Missile)
            .min_by_key(|(_, c)| c.distance_to(self) as u32)
            .map(|(id, _)| *id);
    }
}

////////////////////////////////////////////////////////////////
