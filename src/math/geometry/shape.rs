use oort_api::prelude::Vec2;

use crate::{draw::Colour, math::kinematics::Position};

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// Common interface of 2D shapes.
///
pub trait Shape {
    /// Description
    /// -----------
    /// Translate the shape by the given vector.
    ///
    /// Parmaters
    /// ---------
    /// * `vector` - Vector to translate the ellipse by.
    ///
    fn translate(&mut self, vector: &Vec2);

    /// Description
    /// -----------
    /// Determine if a point lies within a shape.
    ///
    /// Parmaters
    /// ---------
    /// * `point` - Point or object implementing Position. Only tests if the central point of an
    ///             object is contained.
    ///
    fn contains<T: Position>(&self, point: &T) -> bool;

    /// Description
    /// -----------
    /// Return the distance from a point to the closest edge of a shape.
    ///
    /// Parmaters
    /// ---------
    /// * `point` - Point or object implementing Position. The returned distance is measured to the
    ///             objects central point, not it's closest.
    ///
    fn min_distance_to<T: Position>(&self, point: &T) -> f64;

    /// Description
    /// -----------
    /// Return the distance from a point to the furthest edge of a shape.
    ///
    /// Parmaters
    /// ---------
    /// * `point` - Point or object implementing Position. The returned distance is measured to the
    ///             objects central point, not it's furthest.
    ///
    fn max_distance_to<T: Position>(&self, point: &T) -> f64;

    /// Description
    /// -----------
    /// Return the distance from a point to the closest and furthest edge of a shape.
    ///
    /// Parmaters
    /// ---------
    /// * `point` - Point or object implementing Position. The returned distance is measured to the
    ///             objects central point.
    ///
    /// Returns
    /// -------
    /// Tuple where the first element is the smaller distance and the second is the greater.
    ///
    fn minmax_distance_to<T: Position>(&self, point: &T) -> (f64, f64);

    /// Description
    /// -----------
    /// Draw the shape.
    ///
    fn draw(&self, colour: Colour);
}

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// Common interface of 2D shapes that have no verticies.
///
pub trait EllipticalShape: Shape {
    /// Description
    /// -----------
    /// Return the radius of the shape at the given angle.
    ///
    /// Parmaters
    /// ---------
    /// * `angle` - Angle to measure the radius at. Measured from the positive x axis.
    ///
    /// <img src="https://raw.githubusercontent.com/sullyy9/oort-ai/master/docs/images/EllipticalShape_radius.svg" width="426px" height="240px">
    ///
    fn radius(&self, angle: f64) -> f64;

    /// Description
    /// -----------
    /// Expand the shape by the given ammount in all directions.
    ///
    fn expand(&mut self, ammount: f64);
}

////////////////////////////////////////////////////////////////
