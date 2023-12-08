use oort_api::prelude::scenario_name;

////////////////////////////////////////////////////////////////

pub enum Scenario {
    Sandbox,
    FighterDuel,

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
            "fighter_duel" => Self::FighterDuel,
            _ => Self::Unknown,
        }
    }
}

////////////////////////////////////////////////////////////////
