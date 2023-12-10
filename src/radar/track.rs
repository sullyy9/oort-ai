use oort_api::prelude::*;

use crate::math::geometry::Shape;

use super::{
    common::{RadarControl, TrackingRadarControl},
    contacts::{RadarContact, TrackedContact, TrackedRadarContact},
    emitter::Emitter,
    math::kinematics::{Acceleration, Position},
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct TrackingRadar();

////////////////////////////////////////////////////////////////

impl RadarControl for TrackingRadar {}

////////////////////////////////////////////////////////////////

impl TrackingRadar {
    const STANDARD_WIDTH: f64 = std::f64::consts::PI / 32.0;

    pub fn new() -> Self {
        return Self();
    }
}

////////////////////////////////////////////////////////////////

impl TrackingRadarControl for TrackingRadar {
    type Contact = TrackedContact;

    /// Description
    /// -----------
    /// Scan for a contact in the radar beam.
    ///
    fn scan<E: Position>(
        &mut self,
        emitter: &E,
        mut target: Self::Contact,
    ) -> Option<Self::Contact> {
        let emitter = Emitter::new(emitter, self);
        return self.get_scan().map(|s| {
            target.update(&emitter, &s);
            target
        });
    }

    /// Description
    /// -----------
    /// Adjust the radar beam for the next tick.
    ///
    fn adjust<T: Acceleration>(&self, emitter: &T, target: &TrackedContact) {
        let time_elapsed = target.time_elapsed() + TICK_LENGTH;

        let area = target.get_area_after(time_elapsed);
        let (min, max) = area.minmax_distance_to(&emitter.position_after(TICK_LENGTH));

        self.set_width(Self::STANDARD_WIDTH);
        self.set_heading(emitter.bearing_to(target));
        self.set_min_distance(min);
        self.set_max_distance(max);
    }
}

////////////////////////////////////////////////////////////////
