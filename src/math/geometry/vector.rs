use oort_api::prelude::{angle_diff, vec2, Vec2, Vec2Extras};

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// 2D vector type wrapper over Oort's Vec2.
///
#[derive(Default, Debug, Clone, PartialEq)]
pub struct Vector(pub(super) Vec2);

////////////////////////////////////////////////////////////////

/// Description
/// -----------
/// Trait for types that can be treated as a vector.
///
pub trait AsVector {
    /// Description
    /// -----------
    /// Return a vector.
    ///
    fn as_vector(&self) -> Vector;
}

////////////////////////////////////////////////////////////////
// construction / convertion
////////////////////////////////////////////////////////////////

impl Vector {
    pub fn new(x: f64, y: f64) -> Self {
        return Self(vec2(x, y));
    }

    /// Description
    /// -----------
    /// Return a unit vector pointing along the x axis. 
    /// 
    pub fn x_axis() -> Self {
        return Self(vec2(1.0, 0.0));
    }

    /// Description
    /// -----------
    /// Return a unit vector pointing along the y axis. 
    /// 
    pub fn y_axis() -> Self {
        return Self(vec2(0.0, 1.0));
    }
}

///////////////////////¬/////////////////////////////////////////

impl AsVector for Vector {
    fn as_vector(&self) -> Self {
        return self.clone();
    }
}

////////////////////////////////////////////////////////////////

impl From<Vec2> for Vector {
    fn from(vec: Vec2) -> Self {
        return Self(vec);
    }
}

////////////////////////////////////////////////////////////////
// coordinates
////////////////////////////////////////////////////////////////

impl Vector {
    pub fn x(&self) -> f64 {
        return self.0.x;
    }

    pub fn y(&self) -> f64 {
        return self.0.y;
    }
}

////////////////////////////////////////////////////////////////
// operations
////////////////////////////////////////////////////////////////

impl Vector {
    pub fn length(&self) -> f64 {
        return self.0.length();
    }

    pub fn heading(&self) -> f64 {
        return self.0.angle();
    }

    pub fn dot<T: AsVector>(self, other: &T) -> f64 {
        let other = other.as_vector();
        return self.0.dot(other.0);
    }

    pub fn angle_between<T: AsVector>(self, other: &T) -> f64 {
        let other = other.as_vector();
        return angle_diff(self.heading(), other.heading());
    }

    pub fn normalized(&self) -> Self {
        return Self(self.0.normalize());
    }

    pub fn rotated(&self, angle: f64) -> Self {
        return Self(self.0.rotate(angle));
    }
}

////////////////////////////////////////////////////////////////
