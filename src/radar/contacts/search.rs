use oort_api::prelude::*;

use super::{
    emitter::Emitter,
    error::RadarContactError,
    math::{
        geometry::Ellipse,
        kinematics::{Position, Velocity},
    },
    ship::stats::MaxAcceleration,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct SearchContact {
    pub time: f64,
    pub emitter: Emitter,

    pub class: Class,
    pub position: Vec2,
    pub velocity: Vec2,
    pub rssi: f64,
    pub snr: f64,

    pub error: RadarContactError,
}

////////////////////////////////////////////////////////////////

impl SearchContact {
    pub fn new(time: f64, emitter: &Emitter, scan: &ScanResult) -> Self {
        Self {
            time,
            emitter: emitter.clone(),

            class: scan.class,
            position: scan.position,
            velocity: scan.velocity,
            rssi: scan.rssi,
            snr: scan.snr,

            error: RadarContactError::from(scan),
        }
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

impl SearchContact {
    pub fn time_elapsed(&self) -> f64 {
        return current_time() - self.time;
    }

    pub fn is_same_class(&self, other: &Self) -> bool {
        return self.class == other.class;
    }
}

////////////////////////////////////////////////////////////////

impl SearchContact {
    /// Description
    /// -----------
    /// Get the area covering the posible posistions of the contact at a specific point in time
    /// after it was detected.
    ///  
    pub fn get_area_after(&self, time: f64) -> Ellipse {
        let mut area = self.get_initial_area();

        // Move the area according to it's approximate velocity.
        area.translate(&(self.velocity * time));

        // Expand the area to take into account velocity error and possible accleration.
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(self.error.velocity * time + (0.5 * max_accel.magnitude() * time.powf(2.0)));

        return area;
    }

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
