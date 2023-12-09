use oort_api::Class;

use crate::math::kinematics::{Acceleration, Position, Velocity};

////////////////////////////////////////////////////////////////

pub trait RadarContact: Position + Velocity {
    type AreaShape;

    /// Description
    /// -----------
    /// Return the time at which the contact was last updated.
    ///
    /// Returns
    /// -------
    /// Time in seconds.
    ///
    fn time(&self) -> f64;

    /// Description
    /// -----------
    /// Return the time elapsed since the last update. The current time is aquired from oort_api's
    /// current_time() function.
    ///
    /// Returns
    /// -------
    /// Time elapsed in seconds.
    ///
    fn time_elapsed(&self) -> f64;

    /// Description
    /// -----------
    /// Return the ship class of the contact.
    ///
    fn class(&self) -> Class;

    /// Description
    /// -----------
    /// Get the area covering the posible posistions of the contact at a specific instant in time
    /// after it was detected.
    ///  
    fn get_area_after(&self, time: f64) -> Self::AreaShape;

    /// Description
    /// -----------
    /// Get the area covering the posible posistions of the contact at the current instant.
    ///  
    fn get_area_now(&self) -> Self::AreaShape {
        return self.get_area_after(self.time_elapsed());
    }
}

////////////////////////////////////////////////////////////////

pub trait TrackedRadarContact: RadarContact + Acceleration {}

////////////////////////////////////////////////////////////////
