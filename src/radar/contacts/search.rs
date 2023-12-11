use oort_api::prelude::*;

use crate::math::geometry::{Ellipse, EllipticalShape, Shape, Vector};
use crate::math::kinematics::{Position, Velocity};
use crate::ship::stats::MaxAcceleration;

use super::{emitter::Emitter, error::RadarContactError, interface::RadarContact, TrackedContact};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct SearchContact {
    pub(super) time: f64,
    pub(super) emitter: Emitter,

    pub(super) class: Class,
    pub(super) position: Vec2,
    pub(super) velocity: Vec2,
    pub(super) rssi: f64,
    pub(super) snr: f64,

    pub(super) error: RadarContactError,
}

////////////////////////////////////////////////////////////////
/// construction / conversion
////////////////////////////////////////////////////////////////

impl SearchContact {
    pub fn new(time: f64, emitter: &Emitter, scan: &ScanResult) -> Self {
        return Self {
            time,
            emitter: emitter.clone(),

            class: scan.class,
            position: scan.position,
            velocity: scan.velocity,
            rssi: scan.rssi,
            snr: scan.snr,

            error: RadarContactError::from(scan),
        };
    }
}

impl From<TrackedContact> for SearchContact {
    fn from(mut contact: TrackedContact) -> Self {
        // Should be safe as TrackedContact is always constructed with at least one element in each
        // field.
        return Self {
            time: contact.time(),
            emitter: contact.emitter.pop_back().unwrap(),
            class: contact.class(),
            position: contact.position(),
            velocity: contact.velocity(),
            rssi: contact.rssi.pop_back().unwrap(),
            snr: contact.snr.pop_back().unwrap(),
            error: contact.error.pop_back().unwrap(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl Position for SearchContact {
    fn position(&self) -> Vec2 {
        return self.position;
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for SearchContact {
    fn velocity(&self) -> Vec2 {
        return self.velocity;
    }
}

////////////////////////////////////////////////////////////////

impl RadarContact for SearchContact {
    type AreaShape = Ellipse;

    fn time(&self) -> f64 {
        return self.time;
    }

    fn time_elapsed(&self) -> f64 {
        return current_time() - self.time;
    }

    fn class(&self) -> Class {
        return self.class;
    }

    fn get_area_after(&self, time: f64) -> Self::AreaShape {
        let mut area = self.get_initial_area();

        // Move the area according to it's approximate velocity.
        area.translate(&Vector::from(self.velocity * time));

        // Expand the area to take into account velocity error and possible accleration.
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(self.error.velocity * time + (0.5 * max_accel.magnitude() * time.powf(2.0)));

        return area;
    }
}

////////////////////////////////////////////////////////////////

impl SearchContact {
    /// Description
    /// -----------
    /// Get the area in which a radar contact is present at the moment it was detected, taking
    /// into account the error in the position reading.
    ///
    pub fn get_initial_area(&self) -> Ellipse {
        let bearing = self.bearing_to(&self.emitter.position);
        let distance = self.distance_to(&self.emitter.position);

        let width = f64::atan(self.error.bearing) * distance * 2.0;
        let height = self.error.distance * 2.0;
        return Ellipse::new(&self.position, bearing, width, height);
    }
}

////////////////////////////////////////////////////////////////
