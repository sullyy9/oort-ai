use oort_api::prelude::*;

use super::{
    emitter::Emitter,
    error::RadarContactError,
    math::{
        geometry::Ellipse,
        kinematics::{Position, Velocity},
    },
    ship::stats::MaxAcceleration,
    TrackedContact,
};

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
            rssi: contact.rssi(),
            snr: contact.snr(),
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
/// field access
////////////////////////////////////////////////////////////////

#[allow(dead_code)]
impl SearchContact {
    /// Description
    /// -----------
    /// Return information about the radar emitter used to scan the contact.
    ///
    pub fn emitter(&self) -> &Emitter {
        return &self.emitter;
    }

    /// Description
    /// -----------
    /// Return the time at which the contact was last updated.
    ///
    /// Returns
    /// -------
    /// Time in seconds.
    ///
    pub fn time(&self) -> f64 {
        return self.time;
    }

    /// Description
    /// -----------
    /// Return the time elapsed since the last update. The current time is aquired from oort_api's
    /// current_time() function.
    ///
    /// Returns
    /// -------
    /// Time elapsed in seconds.
    ///
    pub fn time_elapsed(&self) -> f64 {
        return current_time() - self.time;
    }

    /// Description
    /// -----------
    /// Return the ship class of the contact.
    ///
    pub fn class(&self) -> Class {
        return self.class;
    }

    /// Description
    /// -----------
    /// Return the received signal strength of the contact.
    ///
    /// Returns
    /// -------
    /// RSSI in dB.
    ///
    pub fn rssi(&self) -> f64 {
        return self.rssi;
    }

    /// Description
    /// -----------
    /// Return the signal to noise ratio of the contact.
    ///
    /// Returns
    /// -------
    /// SNR in dB.
    ///
    pub fn snr(&self) -> f64 {
        return self.snr;
    }

    /// Description
    /// -----------
    /// Return the range of possible error in the contact.
    ///
    pub fn error(&self) -> &RadarContactError {
        return &self.error;
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
