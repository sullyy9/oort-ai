use oort_api::prelude::{Class, Vec2};

use crate::math::{geometry::Ellipse, kinematics::Position};

use super::{interface::RadarContact, SearchContact, TrackedContact, TrackedRadarContact};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub enum Contact<S: RadarContact, T: TrackedRadarContact> {
    Search(S),
    Tracked(T),
}

////////////////////////////////////////////////////////////////

impl<S: RadarContact, T: TrackedRadarContact> Position for Contact<S, T> {
    fn position(&self) -> Vec2 {
        return match self {
            Self::Search(contact) => contact.position(),
            Self::Tracked(contact) => contact.position(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl<S: RadarContact, T: TrackedRadarContact<AreaShape = S::AreaShape>> Contact<S, T> {
    pub fn time_elapsed(&self) -> f64 {
        return match self {
            Self::Search(contact) => contact.time_elapsed(),
            Self::Tracked(contact) => contact.time_elapsed(),
        };
    }

    pub fn class(&self) -> Class {
        return match self {
            Self::Search(contact) => contact.class(),
            Self::Tracked(contact) => contact.class(),
        };
    }

    pub fn get_area_after(&self, time: f64) -> S::AreaShape {
        return match self {
            Self::Search(contact) => contact.get_area_after(time),
            Self::Tracked(contact) => contact.get_area_after(time),
        };
    }
}

////////////////////////////////////////////////////////////////
