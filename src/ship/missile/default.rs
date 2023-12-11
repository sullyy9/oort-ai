use oort_api::prelude::*;

use crate::ship::class::ShipClassLoop;

use super::{
    control::{Rotation, Translation},
    draw::{self, Colour},
    math::{
        kinematics::{Acceleration, AngularVelocity, Heading, KinematicModel, Position, Velocity},
        Intercept,
    },
    radar::{
        board::{ContactBoard, UniqueContactBoard},
        contacts::{Contact, SearchContact, TrackedContact},
        CompositeRadar,
    },
    radio::{Radio, RadioMessage},
    stats::MaxAcceleration,
};

pub struct DefaultMissile {
    radar: CompositeRadar,
    radio: Radio,

    target_position: Option<Vec2>,

    acceleration: Vec2,
}

////////////////////////////////////////////////////////////////

impl Position for DefaultMissile {
    fn position(&self) -> Vec2 {
        return position();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for DefaultMissile {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for DefaultMissile {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Heading for DefaultMissile {
    fn heading(&self) -> f64 {
        return heading();
    }
}

////////////////////////////////////////////////////////////////

impl AngularVelocity for DefaultMissile {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}

////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////

impl Rotation for DefaultMissile {
    fn set_angular_acceleration(&mut self, acceleration: f64) {
        torque(acceleration)
    }
}

////////////////////////////////////////////////////////////////

impl DefaultMissile {
    pub fn new() -> Self {
        let radio = Radio::new();
        let target_position = if let Some(RadioMessage::Position(position)) = radio.receive() {
            let heading = position.angle();
            // radar.set_search_heading(heading - (PI / 8.0), heading + (PI / 8.0));
            Some(position)
        } else {
            None
        };

        return Self {
            radar: CompositeRadar::new(UniqueContactBoard::new()),
            radio,

            target_position,

            acceleration: vec2(0.0, 0.0),
        };
    }
}

impl ShipClassLoop for DefaultMissile {
    fn tick(&mut self) {
        // Update radar contacts.
        self.radar.scan(&self.position());

        // Find a target from amongst the radar contacts.
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
                .filter(|(_, c)| matches!(c, Contact::Search(_)))
                .min_by(|(_, c1), (_, c2)| {
                    c1.distance_to(self)
                        .partial_cmp(&c2.distance_to(self))
                        .unwrap()
                });

            if let Some((&id, _)) = closest_contact {
                if let Err(error) = self.radar.start_tracking(id) {
                    debug!("ERROR - {error:?}")
                }
            }

            None
        };

        // Intercept the target.
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

////////////////////////////////////////////////////////////////
