use oort_api::prelude::*;

#[derive(Clone, PartialEq, PartialOrd, Debug)]
pub struct MaxAcceleration {
    pub forward: f64,
    pub reverse: f64,
    pub lateral: f64,
    pub angular: f64,
}

impl From<Class> for MaxAcceleration {
    fn from(value: Class) -> Self {
        return match value {
            Class::Fighter => Self {
                forward: 60.0,
                reverse: 30.0,
                lateral: 30.0,
                angular: 2.0 * std::f64::consts::PI,
            },
            Class::Frigate => Self {
                forward: 10.0,
                reverse: 5.0,
                lateral: 5.0,
                angular: std::f64::consts::PI / 4.0,
            },
            Class::Cruiser => Self {
                forward: 5.0,
                reverse: 2.5,
                lateral: 2.5,
                angular: std::f64::consts::PI / 8.0,
            },
            Class::Asteroid => Self {
                forward: 0.0,
                reverse: 0.0,
                lateral: 0.0,
                angular: 0.0,
            },
            Class::Target => todo!(),
            Class::Missile => Self {
                forward: 300.0,
                reverse: 0.0,
                lateral: 100.0,
                angular: 4.0 * std::f64::consts::PI,
            },
            Class::Torpedo => Self {
                forward: 70.0,
                reverse: 0.0,
                lateral: 20.0,
                angular: 2.0 * std::f64::consts::PI,
            },
            Class::Unknown => panic!(),
        };
    }
}

impl MaxAcceleration {
    pub fn magnitude(&self) -> f64 {
        let medial = f64::max(self.forward, self.reverse);
        return f64::sqrt(medial.powf(2.0) + self.lateral.powf(2.0));
    }
}
