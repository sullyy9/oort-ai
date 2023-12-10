mod interface;
mod search;
mod tracking;

// Imports
use super::{contacts, emitter};

// Exports.
pub use self::{
    interface::{RadarControl, SearchRadarControl, TrackingRadarControl},
    search::SearchRadar,
    tracking::TrackingRadar,
};
