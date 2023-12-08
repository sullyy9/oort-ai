#![allow(clippy::needless_return)]

use oort_api::prelude::*;

mod control;
mod draw;
mod math;
mod radar;
mod radio;
mod scenario;
mod ship;

use self::{
    scenario::Scenario,
    ship::{
        experimental::{ContactDrawer, RadarTester},
        fighter::{DefaultFighter, Duelist},
        missile::DefaultMissile,
        ShipClass,
    },
};

////////////////////////////////////////////////////////////////

pub struct Ship {
    class: ShipClass,
}

impl Default for Ship {
    fn default() -> Self {
        return Self::from(ShipClass::Unknown());
    }
}

////////////////////////////////////////////////////////////////

impl From<ShipClass> for Ship {
    fn from(class: ShipClass) -> Self {
        return Self { class };
    }
}

////////////////////////////////////////////////////////////////

impl Ship {
    const SANDBOX_MODE: &'static str = "radar_test";

    pub fn new() -> Ship {
        let scenario = Scenario::current();

        // Override for sandbox.
        if matches!(scenario, Scenario::Sandbox) {
            return match Self::SANDBOX_MODE {
                "radar_test" => Self::from(ShipClass::ExRadarTester(RadarTester::new())),
                "contact_draw" => Self::from(ShipClass::ExContactDrawer(ContactDrawer::new())),
                _ => panic!("Error - Unknown sandbox mode"),
            };
        }

        use ShipClass::*;
        return match class() {
            Class::Fighter => match Scenario::current() {
                Scenario::FighterDuel => Self::from(Fighter(Box::new(Duelist::new()))),
                _ => Self::from(Fighter(Box::new(DefaultFighter::new()))),
            },

            Class::Missile => Self::from(Missile(Box::new(DefaultMissile::new()))),

            _ => Self::default(),
        };
    }

    pub fn tick(&mut self) {
        match &mut self.class {
            ShipClass::Fighter(fighter) => fighter.tick(),
            ShipClass::Missile(missile) => missile.tick(),
            ShipClass::ExContactDrawer(contact_drawer) => contact_drawer.tick(),
            ShipClass::ExRadarTester(radar_tester) => radar_tester.tick(),

            ShipClass::Unknown() => (),
        }
    }
}

////////////////////////////////////////////////////////////////
