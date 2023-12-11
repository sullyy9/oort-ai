use std::collections::VecDeque;

use oort_api::prelude::*;

use crate::math::geometry::{Ellipse, EllipticalShape, Shape, Vector};
use crate::math::kinematics::{Acceleration, Position, Velocity};
use crate::ship::stats::MaxAcceleration;

use super::{
    emitter::Emitter, error::RadarContactError, RadarContact, SearchContact, TrackedRadarContact,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct TrackedContact {
    pub(super) emitter: VecDeque<Emitter>,
    pub(super) time: VecDeque<f64>,

    pub(super) class: Class,
    pub(super) position: VecDeque<Vec2>,
    pub(super) velocity: VecDeque<Vec2>,
    pub(super) rssi: VecDeque<f64>,
    pub(super) snr: VecDeque<f64>,

    pub(super) error: VecDeque<RadarContactError>,
    pub(super) acceleration: Vec2,
}

////////////////////////////////////////////////////////////////
/// constants
////////////////////////////////////////////////////////////////

impl TrackedContact {
    const MAX_DATA_POINTS: usize = 9;
}

////////////////////////////////////////////////////////////////
/// construction / conversion
////////////////////////////////////////////////////////////////

#[allow(dead_code)]
impl TrackedContact {
    pub fn new(scan: &ScanResult, scan_emitter: &Emitter) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(scan_emitter.clone());

        let mut time = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        time.push_back(current_time());

        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(scan.position);

        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(scan.velocity);

        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(scan.rssi);

        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(scan.snr);

        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(RadarContactError::from(scan));

        return Self {
            emitter,
            time,

            class: scan.class,
            position,
            velocity,
            rssi,
            snr,

            error,
            acceleration: Vec2::default(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl From<SearchContact> for TrackedContact {
    fn from(contact: SearchContact) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(contact.emitter);

        let mut time = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        time.push_back(contact.time);

        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(contact.position);

        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(contact.velocity);

        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(contact.rssi);

        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(contact.snr);

        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(contact.error);

        return Self {
            emitter,
            time,

            class: contact.class,
            position,
            velocity,
            rssi,
            snr,

            error,
            acceleration: Vec2::default(),
        };
    }
}

impl From<&SearchContact> for TrackedContact {
    fn from(contact: &SearchContact) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(contact.emitter.clone());

        let mut time = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        time.push_back(contact.time);

        let mut position = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        position.push_back(contact.position);

        let mut velocity = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        velocity.push_back(contact.velocity);

        let mut rssi = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        rssi.push_back(contact.rssi);

        let mut snr = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        snr.push_back(contact.snr);

        let mut error = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        error.push_back(contact.error.clone());

        return Self {
            emitter,
            time,

            class: contact.class,
            position,
            velocity,
            rssi,
            snr,

            error,
            acceleration: Vec2::default(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl Position for TrackedContact {
    fn position(&self) -> Vec2 {
        return *self.position.back().unwrap();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for TrackedContact {
    fn velocity(&self) -> Vec2 {
        return *self.velocity.back().unwrap();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for TrackedContact {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl RadarContact for TrackedContact {
    type AreaShape = Ellipse;

    fn time(&self) -> f64 {
        // Should be safe as type is always constructed with at least one entry in this field.
        return *self.time.back().unwrap();
    }

    fn time_elapsed(&self) -> f64 {
        // Should be safe as type is always constructed with at least one entry in this field.
        return current_time() - self.time.back().unwrap();
    }

    fn class(&self) -> Class {
        return self.class;
    }

    fn get_area_after(&self, time: f64) -> Self::AreaShape {
        let mut area = self.get_initial_area();

        // Move the area according to it's approximate velocity.
        area.translate(&Vector::from(*self.velocity.back().unwrap() * time));

        // Expand the area to take into account velocity error and possible accleration.
        // TODO: should be using the actual acceleration here.
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(
            self.error.back().unwrap().velocity * time
                + (0.5 * max_accel.magnitude() * time.powf(2.0)),
        );

        return area;
    }
}

////////////////////////////////////////////////////////////////

impl TrackedRadarContact for TrackedContact {
    fn update(&mut self, emitter: &Emitter, scan: &ScanResult) {
        if self.emitter.len() == Self::MAX_DATA_POINTS {
            self.emitter.pop_front();
        }
        self.emitter.push_back(emitter.clone());

        if self.time.len() == Self::MAX_DATA_POINTS {
            self.time.pop_front();
        }
        self.time.push_back(current_time());

        if self.position.len() == Self::MAX_DATA_POINTS {
            self.position.pop_front();
        }
        self.position.push_back(scan.position);

        if self.velocity.len() == Self::MAX_DATA_POINTS {
            self.velocity.pop_front();
        }
        self.velocity.push_back(scan.velocity);

        if self.rssi.len() == Self::MAX_DATA_POINTS {
            self.rssi.pop_front();
        }
        self.rssi.push_back(scan.rssi);

        if self.snr.len() == Self::MAX_DATA_POINTS {
            self.snr.pop_front();
        }
        self.snr.push_back(scan.snr);

        if self.error.len() == Self::MAX_DATA_POINTS {
            self.error.pop_front();
        }
        self.error.push_back(RadarContactError::from(scan));

        // Find the average change in velocity to estimate the acceleration.
        let mut sum = vec2(0.0, 0.0);

        let records = std::iter::zip(self.velocity.iter(), self.time.iter());
        for (r1, r2) in std::iter::zip(records.clone(), records.skip(1)) {
            let vdelta = r1.0 - r2.0;
            let tdelta = r1.1 - r2.1;
            let accel = vdelta / tdelta;
            sum += accel;
        }

        self.acceleration = sum / (self.velocity.len() - 1) as f64;
    }
}

////////////////////////////////////////////////////////////////

impl TrackedContact {
    /// Description
    /// -----------
    /// Get the area in which a radar contact is present at the moment it was detected, taking
    /// into account the error in the position reading.
    ///
    pub fn get_initial_area(&self) -> Ellipse {
        let emitter = self.emitter.back().unwrap();
        let bearing = self.bearing_to(&emitter.position);
        let distance = self.distance_to(&emitter.position);

        let width = f64::atan(self.error.back().unwrap().bearing) * distance * 2.0;
        let height = self.error.back().unwrap().distance * 2.0;

        return Ellipse::new(self.position.back().unwrap(), bearing, width, height);
    }
}

////////////////////////////////////////////////////////////////
