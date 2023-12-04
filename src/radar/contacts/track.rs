use std::collections::VecDeque;

use oort_api::prelude::*;

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
    emitter: VecDeque<Emitter>,
    timestamp: VecDeque<f64>,

    pub class: Class,
    position: VecDeque<Vec2>,
    velocity: VecDeque<Vec2>,
    rssi: VecDeque<f64>,
    snr: VecDeque<f64>,

    error: VecDeque<RadarContactError>,
    acceleration: Vec2,
}

////////////////////////////////////////////////////////////////

impl TrackedContact {
    const MAX_DATA_POINTS: usize = 9;

    pub fn new(scan: &ScanResult, scan_emitter: &Emitter) -> Self {
        let mut emitter = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        emitter.push_back(scan_emitter.clone());

        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(current_time());

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
            timestamp,

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

        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(contact.time);

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
            timestamp,

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

        let mut timestamp = VecDeque::with_capacity(Self::MAX_DATA_POINTS);
        timestamp.push_back(contact.time);

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
            timestamp,

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

        if self.timestamp.len() == Self::MAX_DATA_POINTS {
            self.timestamp.pop_front();
        }
        self.timestamp.push_back(current_time());

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

        let records = std::iter::zip(self.velocity.iter(), self.timestamp.iter());
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
    pub fn time_elapsed(&self) -> f64 {
        return current_time() - self.timestamp.back().unwrap();
    }

    pub fn is_same_class(&self, other: &Self) -> bool {
        return self.class == other.class;
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

        // // Translate the initial area by velocity.
        // let [p1, p2, p3, p4] = self
        //     .get_initial_area()
        //     .get_verticies()
        //     .map(|v| v + (*self.velocity.back().unwrap() * time));

        // // Expand the boundry of the area by the velocity error and potential acceleration.
        // let max_accel = MaxAcceleration::from(self.class);
        // let radius = self.error.back().unwrap().velocity * time
        //     + (0.5 * max_accel.magnitude() * time.powf(2.0));
        // let radius_vec = vec2(radius, 0.0);

        // let mut verticies: [Vec2; 20] = Default::default();

        // for (i, points) in [&p4, &p1, &p2, &p3, &p4, &p1].windows(3).enumerate() {
        //     let (last, this, next) = (points[0], points[1], points[2]);

        //     let px1 = this + radius_vec.rotate(this.bearing_to(last) + (PI / 2.0));
        //     let px5 = this + radius_vec.rotate(next.bearing_to(this) + (PI / 2.0));
        //     let px3 = this + radius_vec.rotate(px5.bearing_to(&px1) + (PI / 2.0));
        //     let px2 = this + radius_vec.rotate(px3.bearing_to(&px1) + (PI / 2.0));
        //     let px4 = this + radius_vec.rotate(px5.bearing_to(&px3) + (PI / 2.0));

        //     let i = i * 5;
        //     verticies[i] = px1;
        //     verticies[i + 1] = px2;
        //     verticies[i + 2] = px3;
        //     verticies[i + 3] = px4;
        //     verticies[i + 4] = px5;
        // }

        // return Polygon::from(verticies);
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

        let error = self.error.back().unwrap();
        let max_distance = distance + error.distance;
        let min_distance = distance - error.distance;
        let max_bearing = bearing + error.bearing;
        let min_bearing = bearing - error.bearing;

        // Clamp distance and bearing.
        let max_distance = max_distance.clamp(emitter.min_distance, emitter.max_distance);
        let min_distance = min_distance.clamp(emitter.min_distance, emitter.max_distance);

        let max_bearing = max_bearing.clamp(emitter.get_min_heading(), emitter.get_max_heading());
        let min_bearing = min_bearing.clamp(emitter.get_min_heading(), emitter.get_max_heading());

        let width = f64::atan(self.error.back().unwrap().bearing) * distance * 2.0;
        let height = self.error.back().unwrap().distance * 2.0;

        return Ellipse::new(self.position.back().unwrap(), bearing, width, height);

        // Create the polygon.
        // return Polygon::from([
        //     emitter.position + vec2(min_distance, 0.0).rotate(min_bearing),
        //     emitter.position + vec2(min_distance, 0.0).rotate(max_bearing),
        //     emitter.position + vec2(max_distance, 0.0).rotate(max_bearing),
        //     emitter.position + vec2(max_distance, 0.0).rotate(min_bearing),
        // ]);
    }
}

////////////////////////////////////////////////////////////////
