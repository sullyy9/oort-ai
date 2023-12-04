use super::experimental::{ContactDrawer, RadarTester};

////////////////////////////////////////////////////////////////

pub trait ShipClassLoop {
    fn tick(&mut self);
}

////////////////////////////////////////////////////////////////

pub enum ShipClass {
    Fighter(Box<dyn ShipClassLoop>),
    Missile(Box<dyn ShipClassLoop>),

    ExContactDrawer(ContactDrawer),
    ExRadarTester(RadarTester),

    Unknown(),
}

////////////////////////////////////////////////////////////////
