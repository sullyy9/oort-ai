use oort_api::prelude::{Class, Vec2};

use crate::math::kinematics::Position;

use super::{
    math::geometry::{Ellipse, Polygon},
    SearchContact, TrackedContact,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub enum Contact {
    Scanned(SearchContact),
    Tracked(TrackedContact),
}

////////////////////////////////////////////////////////////////

impl Position for Contact {
    fn position(&self) -> Vec2 {
        return match self {
            Self::Scanned(contact) => contact.position(),
            Self::Tracked(contact) => contact.position(),
        };
    }
}

////////////////////////////////////////////////////////////////

impl Contact {
    pub fn time_elapsed(&self) -> f64 {
        return match self {
            Self::Scanned(contact) => contact.time_elapsed(),
            Self::Tracked(contact) => contact.time_elapsed(),
        };
    }

    pub fn is_class(&self, class: Class) -> bool {
        return match self {
            Self::Scanned(contact) => contact.class == class,
            Self::Tracked(contact) => contact.class() == class,
        };
    }

    pub fn get_class(&self) -> Class {
        return match self {
            Self::Scanned(contact) => contact.class,
            Self::Tracked(contact) => contact.class(),
        };
    }

    pub fn get_area_after(&self, time: f64) -> Ellipse {
        return match self {
            Self::Scanned(contact) => contact.get_area_after(time),
            Self::Tracked(contact) => contact.get_area_after(time),
        };
    }
}

////////////////////////////////////////////////////////////////
