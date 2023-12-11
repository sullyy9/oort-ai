use oort_api::prelude::{vec2, Vec2, Vec2Extras};

use super::vector::{AsVector, Vector};

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// 2D point type wrapper over Oort's Vec2.
///
#[derive(Default, Debug, Clone, PartialEq)]
pub struct Point(Vec2);

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// Trait for types that can be treated as a point.
///
pub trait AsPoint {
    /// Description
    /// -----------
    /// Return a point at the objects position. If the object has area, then the point should
    /// typically be at it's centre.
    ///
    fn as_point(&self) -> Point;
}

////////////////////////////////////////////////////////////////
// construction / convertion
////////////////////////////////////////////////////////////////

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        return Self(vec2(x, y));
    }

    pub fn origin() -> Self {
        return Self(vec2(0.0, 0.0));
    }
}

////////////////////////////////////////////////////////////////

impl AsPoint for Point {
    fn as_point(&self) -> Point {
        return self.clone();
    }
}

////////////////////////////////////////////////////////////////

impl From<Vec2> for Point {
    fn from(vec: Vec2) -> Self {
        return Self(vec);
    }
}

impl From<Point> for Vec2 {
    fn from(value: Point) -> Self {
        return value.0;
    }
}

////////////////////////////////////////////////////////////////
// operations
////////////////////////////////////////////////////////////////

impl Point {
    pub fn vector_to<T: AsPoint>(&self, other: &T) -> Vector {
        let other = other.as_point();
        return Vector(other.0 - self.0);
    }

    pub fn vector_from<T: AsPoint>(&self, other: &T) -> Vector {
        let other = other.as_point();
        return Vector(self.0 - other.0);
    }

    pub fn distance_to<T: AsPoint>(&self, other: &T) -> f64 {
        let other = other.as_point();
        return self.0.distance(other.0);
    }

    pub fn translated_by<T: AsVector>(&self, vector: &T) -> Self {
        let vector = vector.as_vector();
        return Self(self.0 + vector.0);
    }
}

////////////////////////////////////////////////////////////////
