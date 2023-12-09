use std::collections::VecDeque;

use oort_api::prelude::*;

use crate::math::geometry::{EllipticalShape, Shape};

use super::{
    emitter::Emitter,
    error::RadarContactError,
    math::{
        geometry::Ellipse,
        kinematics::{Acceleration, Position, Velocity},
    },
    ship::stats::MaxAcceleration,
    SearchContact,
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

impl TrackedContact {
    pub fn update(&mut self, contact: ScanResult, emitter: &Emitter) {
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
        self.position.push_back(contact.position);

        if self.velocity.len() == Self::MAX_DATA_POINTS {
            self.velocity.pop_front();
        }
        self.velocity.push_back(contact.velocity);

        if self.rssi.len() == Self::MAX_DATA_POINTS {
            self.rssi.pop_front();
        }
        self.rssi.push_back(contact.rssi);

        if self.snr.len() == Self::MAX_DATA_POINTS {
            self.snr.pop_front();
        }
        self.snr.push_back(contact.snr);

        if self.error.len() == Self::MAX_DATA_POINTS {
            self.error.pop_front();
        }
        self.error.push_back(RadarContactError::from(contact));

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
/// field access
////////////////////////////////////////////////////////////////

#[allow(dead_code)]
impl TrackedContact {
    /// Description
    /// -----------
    /// Return information about the radar emitter used to scan the contact.
    ///
    pub fn emitter(&self) -> &Emitter {
        // Should be safe as type is always constructed with at least one entry in this field.
        return self.emitter.back().unwrap();
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
        // Should be safe as type is always constructed with at least one entry in this field.
        return *self.time.back().unwrap();
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
        return current_time() - self.time.back().unwrap();
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
        // Should be safe as type is always constructed with at least one entry in this field.
        return *self.rssi.back().unwrap();
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
        // Should be safe as type is always constructed with at least one entry in this field.
        return *self.snr.back().unwrap();
    }

    /// Description
    /// -----------
    /// Return the range of possible error in the contact.
    ///
    pub fn error(&self) -> &RadarContactError {
        // Should be safe as type is always constructed with at least one entry in this field.
        return self.error.back().unwrap();
    }
}

////////////////////////////////////////////////////////////////

impl TrackedContact {
    /// Description
    /// -----------
    /// Get the area covering the posible posistions of the contact at a specific point in time
    /// after it was detected.
    ///
    /// The area looks like a trapezoid with rounded corners.
    /// The possibility that the contact is at any point within the area is not uniform. It is more
    /// likely towards the centre. Due to the combination of distance and bearing error probability
    /// it is least likely in the corners. Area would be perhaps be better represented by a elipse.
    ///
    /// TODO Would an elipse be a better representation (see above)? Allow user to define minimum
    /// probability when creating.
    ///  
    pub fn get_area_after(&self, time: f64) -> Ellipse {
        let mut area = self.get_initial_area();

        // Move the area according to it's approximate velocity.
        area.translate(&(*self.velocity.back().unwrap() * time));

        // Expand the area to take into account velocity error and possible accleration.
        let max_accel = MaxAcceleration::from(self.class);
        area.expand(
            self.error.back().unwrap().velocity * time
                + (0.5 * max_accel.magnitude() * time.powf(2.0)),
        );

        return area;
    }

    /// Description
    /// -----------
    /// Fet the polygon describing the area in which a radar contact is present at the moment it was
    /// detected. This takes into account the error in the position reading.
    ///
    /// The top and bottom of the resulting shape should be arcs but straight lines are used here
    /// instead.
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
