mod board;
mod contact;
mod error;
mod search;
mod track;

// Imports
use super::{draw, emitter, math, ship};

// Exports.
pub use self::{
    board::ContactBoard, contact::Contact, search::SearchContact, track::TrackedContact,
};
