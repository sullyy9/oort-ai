use oort_api::prelude::*;

/// Dimensions of a ship.
/// Each dimension is given from the centre.
#[derive(Clone, PartialEq, PartialOrd, Debug)]
pub struct Dimensions {
    pub fore: f64,
    pub aft: f64,
    pub left: f64,
    pub right: f64,
}

impl From<Class> for Dimensions {
    fn from(value: Class) -> Self {
        return match value {
            Class::Fighter => Self {
                fore: 12.0,
                aft: 6.0,
                left: 8.0,
                right: 8.0,
            },
            Class::Frigate => todo!(),
            Class::Cruiser => todo!(),
            Class::Asteroid => todo!(),
            Class::Target => todo!(),
            Class::Missile => Self {
                fore: 5.0,
                aft: 1.0,
                left: 3.0,
                right: 3.0,
            },
            Class::Torpedo => todo!(),
            Class::Unknown => todo!(),
        };
    }
}

impl Dimensions {
    pub fn length(&self) -> f64 {
        return self.fore + self.aft;
    }

    pub fn width(&self) -> f64 {
        return self.left + self.right;
    }

    pub fn longest(&self) -> f64 {
        return f64::max(
            f64::max(self.fore, self.aft),
            f64::max(self.left, self.right),
        );
    }
}
