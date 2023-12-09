use oort_api::prelude::{current_time, TICK_LENGTH};

use crate::math::geometry::Shape;

use super::{
    common::Radar,
    contacts::SearchContact,
    emitter::Emitter,
    math::kinematics::{Acceleration, Position},
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct ScanningRadar {
    last_heading: f64,
    last_contact: Option<SearchContact>,
}

////////////////////////////////////////////////////////////////

impl Radar for ScanningRadar {}

////////////////////////////////////////////////////////////////

impl ScanningRadar {
    pub fn new() -> Self {
        return Self {
            last_heading: 0.0,
            last_contact: None,
        };
    }
}

////////////////////////////////////////////////////////////////

impl ScanningRadar {
    const STANDARD_WIDTH: f64 = std::f64::consts::PI / 8.0;

    /// Description
    /// -----------
    /// Scan for a contact in the radar beam.
    ///
    pub fn scan<T: Position>(&mut self, emitter: &T) -> Option<SearchContact> {
        let emitter = Emitter::new(emitter, self);

        let contact = self
            .get_scan()
            .map(|s| SearchContact::new(current_time(), &emitter, &s));

        self.last_heading = self.get_heading();
        self.last_contact = contact.clone();

        return contact;
    }

    /// Description
    /// -----------
    /// Adjust the radar beam for the next tick.
    ///
    pub fn adjust<T: Acceleration>(&self, emitter: &T) {
        self.set_width(Self::STANDARD_WIDTH);

        // If we picked up a contact on the last scan then this time, scan behind them.
        if let Some(contact) = &self.last_contact {
            let area = contact.get_area_after(contact.time_elapsed() + TICK_LENGTH);
            let furthest_point = area.max_distance_to(&emitter.position_after(TICK_LENGTH));

            self.set_heading(self.last_heading);
            self.set_min_distance(furthest_point);
            self.set_max_distance(Self::MAX_RADAR_RANGE);
        } else {
            let next_heading = self.last_heading + Self::STANDARD_WIDTH;

            self.set_heading(next_heading);
            self.set_min_distance(0.0);
            self.set_max_distance(Self::MAX_RADAR_RANGE);
        }
    }
}

////////////////////////////////////////////////////////////////
