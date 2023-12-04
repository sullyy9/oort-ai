use oort_api::prelude::scenario_name;

////////////////////////////////////////////////////////////////

pub enum Scenario {
    Sandbox,
    FighterDual,

    Unknown,
}

////////////////////////////////////////////////////////////////

impl Scenario {
    /// Description
    /// -----------
    /// Return the current scenario.
    ///
    pub fn current() -> Self {
        match scenario_name() {
            "sandbox" => Self::Sandbox,
            "fighter_dual" => Self::FighterDual,
            _ => Self::Unknown,
        }
    }
}

////////////////////////////////////////////////////////////////
