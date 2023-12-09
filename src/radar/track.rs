use oort_api::prelude::*;

use crate::math::geometry::Shape;

use super::{
    common::Radar,
    contacts::{Contact, TrackedContact},
    emitter::Emitter,
    math::kinematics::{Acceleration, Position},
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct TrackingRadar();

////////////////////////////////////////////////////////////////

impl Radar for TrackingRadar {}

////////////////////////////////////////////////////////////////

impl TrackingRadar {
    pub fn new() -> Self {
        return Self();
    }
}

////////////////////////////////////////////////////////////////

impl TrackingRadar {
    const STANDARD_WIDTH: f64 = std::f64::consts::PI / 32.0;

    /// Description
    /// -----------
    /// Scan for a contact in the radar beam.
    ///
    pub fn scan<T: Position>(&mut self, target: Contact, emitter: &T) -> Option<TrackedContact> {
        let mut target = match target {
            Contact::Search(contact) => TrackedContact::from(contact),
            Contact::Tracked(contact) => contact,
        };

        let emitter = Emitter::new(emitter, self);
        return self.get_scan().map(|s| {
            target.update(s, &emitter);
            target
        });
    }

    /// Description
    /// -----------
    /// Adjust the radar beam for the next tick.
    ///
    pub fn adjust<T: Acceleration>(&self, target: &TrackedContact, emitter: &T) {
        let time_elapsed = target.time_elapsed() + TICK_LENGTH;

        let area = target.get_area_after(time_elapsed);
        let (min, max) = area.minmax_distance_to(&emitter.position_after(TICK_LENGTH));

        debug!("centre: {}", area.position());
        debug!("range:  {} - {}", min, max);

        self.set_width(Self::STANDARD_WIDTH);
        self.set_heading(emitter.bearing_to(target));
        self.set_min_distance(min);
        self.set_max_distance(max);
    }
}

////////////////////////////////////////////////////////////////
