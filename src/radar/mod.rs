pub mod board;
mod composite;
pub mod contacts;
mod control;
mod emitter;

// Imports
use super::{draw, math, ship};

use self::{
    board::UniqueContactBoard,
    control::{SearchRadar, TrackingRadar},
};

// Exports
pub type CompositeRadar = composite::CompositeRadar<
    SearchRadar,
    TrackingRadar,
    UniqueContactBoard<
        <control::SearchRadar as control::SearchRadarControl>::Contact,
        <control::TrackingRadar as control::TrackingRadarControl>::Contact,
    >,
>;
