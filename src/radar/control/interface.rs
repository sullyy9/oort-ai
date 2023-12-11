use oort_api::prelude::*;

use crate::{
    math::kinematics::{Acceleration, Position},
    radar::contacts::{RadarContact, TrackedRadarContact},
};

////////////////////////////////////////////////////////////////

pub trait RadarControl {
    const MAX_RADAR_RANGE: f64 = 25000.0;

    fn set_heading(&self, heading: f64) {
        set_radar_heading(heading);
    }

    fn get_heading(&self) -> f64 {
        return radar_heading();
    }

    fn set_width(&self, heading: f64) {
        set_radar_width(heading);
    }

    fn get_width(&self) -> f64 {
        return radar_width();
    }

    fn set_min_distance(&self, distance: f64) {
        set_radar_min_distance(distance);
    }

    fn get_min_distance(&self) -> f64 {
        return radar_min_distance();
    }

    fn set_max_distance(&self, distance: f64) {
        set_radar_max_distance(distance);
    }

    fn get_max_distance(&self) -> f64 {
        return radar_max_distance();
    }

    fn get_scan(&self) -> Option<ScanResult> {
        return scan();
    }
}

////////////////////////////////////////////////////////////////

pub trait SearchRadarControl: Default {
    type Contact: RadarContact;

    fn scan<T: Position>(&mut self, emitter: &T) -> Option<Self::Contact>;
    fn adjust<T: Acceleration>(&self, emitter: &T);
}

////////////////////////////////////////////////////////////////

pub trait TrackingRadarControl: Default {
    type Contact: TrackedRadarContact;

    fn scan<T: Position>(&mut self, emitter: &T, target: Self::Contact) -> Option<Self::Contact>;
    fn adjust<T: Acceleration>(&self, emitter: &T, target: &Self::Contact);
}

////////////////////////////////////////////////////////////////
