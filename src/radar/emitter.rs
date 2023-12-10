use oort_api::prelude::*;

use super::{common::RadarControl, math::kinematics::Position};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct EmitterRange {
    pub min: f64,
    pub max: f64,
}

#[derive(Clone, PartialEq, Debug)]
pub struct Emitter {
    pub position: Vec2,
    pub min_distance: f64,
    pub max_distance: f64,
    pub heading: f64,
    pub width: f64,
}

////////////////////////////////////////////////////////////////

impl Emitter {
    /// Description
    /// -----------
    /// Return a new Emitter describing the given ships radar.
    ///
    pub fn new<T: Position, R: RadarControl>(emitter: &T, radar: &R) -> Self {
        return Self {
            position: emitter.position(),
            min_distance: radar.get_min_distance(),
            max_distance: radar.get_max_distance(),
            heading: radar.get_heading(),
            width: radar.get_width(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl Emitter {
    pub fn get_min_heading(&self) -> f64 {
        return self.heading - (self.width / 2.0);
    }

    pub fn get_max_heading(&self) -> f64 {
        return self.heading + (self.width / 2.0);
    }
}

////////////////////////////////////////////////////////////////
