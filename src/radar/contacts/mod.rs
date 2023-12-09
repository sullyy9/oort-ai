mod contact;
mod error;
mod interface;
mod search;
mod track;

// Imports
use super::emitter;

// Exports.
pub use self::{
    contact::Contact,
    interface::{RadarContact, TrackedRadarContact},
    search::SearchContact,
    track::TrackedContact,
};
