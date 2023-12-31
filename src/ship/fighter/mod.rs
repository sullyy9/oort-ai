//! Description
//! -----------
//! Common and role specific fighter behaviour.
//!

mod default;
mod duelist;

// Imports.
use super::{class, control, draw, math, radar, radio};

// Exports.
pub use self::{default::DefaultFighter, duelist::Duelist};
