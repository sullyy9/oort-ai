use oort_api::prelude::*;

use super::{draw, kinematics::Position};

////////////////////////////////////////////////////////////////

pub struct Ellipse {
    centre: Vec2,
    orientation: f64,
    width: f64,
    height: f64,
}

////////////////////////////////////////////////////////////////

impl Position for Ellipse {
    fn position(&self) -> Vec2 {
        return self.centre;
    }
}

////////////////////////////////////////////////////////////////

impl Ellipse {
    /// Description
    /// -----------
    /// Create a new ellipse.
    /// Since +x axis is heading 0, height is size along the x axis.
    ///
    pub fn new<T: Position>(centre: &T, orientation: f64, width: f64, height: f64) -> Self {
        // Make them allways be W I D E B O I S. That way we know which axis to iterate over when
        // drawing.
        return if width > height {
            Self {
                centre: centre.position(),
                orientation: orientation + std::f64::consts::FRAC_PI_2,
                width: height,
                height: width,
            }
        } else {
            return Self {
                centre: centre.position(),
                orientation,
                width,
                height,
            };
        };
    }
}

////////////////////////////////////////////////////////////////
/// Manipulation.
////////////////////////////////////////////////////////////////

impl Ellipse {
    /// Description
    /// -----------
    /// Translate the ellipse by the given ammount.
    ///
    /// Parmaters
    /// ---------
    /// * `vector` - Ammount to translate the ellipse by.
    ///
    pub fn translate(&mut self, vector: &Vec2) {
        self.centre += vector;
    }

    /// Description
    /// -----------
    /// Expand the ellipse by the given ammount in all directions.
    ///
    /// Parmaters
    /// ---------
    /// * `value` - Ammount to expand the ellipse by.
    ///
    pub fn expand(&mut self, value: f64) {
        self.width += value * 2.0;
        self.height += value * 2.0;
    }
}

////////////////////////////////////////////////////////////////

impl Ellipse {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        let point = point.position_relative_to(self);
        let point = point.rotate(-self.orientation);

        let a = self.height / 2.0;
        let b = self.width / 2.0;

        let xpart = (point.x).powi(2) / a.powi(2);
        let ypart = (point.y).powi(2) / b.powi(2);
        return (xpart + ypart) <= 1.0;
    }

    pub fn max_distance_to<T: Position>(&self, point: &T) -> f64 {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;

        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );

        println!("angle: {}", angle);
        println!("radius: {}", radius);
        debug!("angle: {}", angle);
        debug!("radius: {}", radius);
        return centre_distance + radius;
    }

    pub fn min_distance_to<T: Position>(&self, point: &T) -> f64 {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;

        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );

        return (centre_distance - radius).abs();
    }

    pub fn minmax_distance_to<T: Position>(&self, point: &T) -> (f64, f64) {
        let centre_distance = point.distance_to(&self.centre);
        let angle = point.bearing_to(&self.centre) - self.orientation;

        let a = self.height / 2.0;
        let b = self.width / 2.0;
        let radius = (a * b)
            / f64::sqrt(
                (a.powi(2) * f64::sin(angle).powi(2)) + (b.powi(2) * f64::cos(angle).powi(2)),
            );

        return ((centre_distance - radius).abs(), centre_distance + radius);
    }
}

////////////////////////////////////////////////////////////////

impl Ellipse {
    pub fn draw(&self, colour: draw::Colour) {
        let a = self.height / 2.0;
        let b = self.width / 2.0;

        // We know its a W I D E B O I so we can just use x as the iteration axis.
        let mut points = Vec::new();

        // Iterate over x.
        // y = sqrt(b^2 - (x^2 / a^2)b^2)
        const POINTS: usize = 40;
        let step = self.height / (POINTS / 2) as f64;

        let y = |x: f64| f64::sqrt(b.powf(2.0) - ((x.powf(2.0) / a.powf(2.0)) * b.powf(2.0)));

        for i in 0..(POINTS / 2) {
            let x = -a + (i as f64 * step);
            points.push(vec2(x, y(x)));
        }
        points.push(vec2(a, y(a)));

        // So far we only have half of the shape but the other half is just a reflection over the
        // x axis.
        for i in (1..(points.len() - 1)).rev() {
            let p = &points[i];
            points.push(vec2(p.x, -p.y));
        }

        // So far the shape has been constructed without it's correct orrientation and around the
        // origin. Rotate and move it into the correct position.
        points
            .iter_mut()
            .for_each(|p| *p = p.rotate(self.orientation));
        points.iter_mut().for_each(|p| *p += self.centre);

        // Draw the shape.
        for pair in points.windows(2) {
            let (p1, p2) = (&pair[0], &pair[1]);
            draw::line(p1, p2, colour);
        }
        draw::line(points.first().unwrap(), points.last().unwrap(), colour);
    }
}

////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_contains() {
        let ellipse = Ellipse::new(&vec2(0.0, 0.0), 0.0, 5.0, 10.0);

        assert!(ellipse.contains(&vec2(4.0, 0.0)));
        assert!(ellipse.contains(&vec2(0.0, 2.0)));

        assert!(!ellipse.contains(&vec2(0.0, 3.0)));
        assert!(!ellipse.contains(&vec2(0.0, -6.0)));
        assert!(!ellipse.contains(&vec2(5.0, -6.0)));
        assert!(!ellipse.contains(&vec2(0.0, 6.0)));
        assert!(!ellipse.contains(&vec2(6.0, 0.0)));

        let ellipse = Ellipse::new(&vec2(0.0, 0.0), std::f64::consts::FRAC_PI_4, 5.0, 10.0);
        assert!(ellipse.contains(&vec2(3.0, 3.0)));
        assert!(!ellipse.contains(&vec2(-3.0, 3.0)));
    }

    #[test]
    fn test_max_distance_to() {
        let ellipse = Ellipse::new(&vec2(0.0, 5.0), 0.0, 5.0, 10.0);

        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 0.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 10.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&vec2(5.0, 5.0)), 10.0);
        assert_eq!(ellipse.max_distance_to(&vec2(-5.0, 5.0)), 10.0);

        // Questionable behaviour when point is at the centre of the ellipse.
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 5.0)), 5.0);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 6.0)), 3.5);

        // 45 degree angle with tip touching the origin.
        let height = f64::sqrt(5.0_f64.powi(2) + 5.0_f64.powi(2)) * 2.0;
        let ellipse = Ellipse::new(&vec2(5.0, 5.0), std::f64::consts::FRAC_PI_4, 50.0, height);

        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 0.0)), height);
        assert_eq!(ellipse.max_distance_to(&vec2(10.0, 10.0)), height);

        // Above but mirrored.
        let angle = std::f64::consts::FRAC_PI_4 + std::f64::consts::FRAC_PI_2;
        let ellipse = Ellipse::new(&vec2(5.0, 5.0), angle, 50.0, height);
        assert_eq!(ellipse.max_distance_to(&vec2(0.0, 10.0)), height);
    }

    #[test]
    fn test_min_distance_to() {
        let ellipse = Ellipse::new(&vec2(5.0, 0.0), std::f64::consts::FRAC_PI_2, 5.0, 10.0);

        assert_eq!(ellipse.min_distance_to(&vec2(0.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(10.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(5.0, 5.0)), 0.0);
        assert_eq!(ellipse.min_distance_to(&vec2(5.0, -5.0)), 0.0);

        assert_eq!(ellipse.min_distance_to(&vec2(5.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&vec2(5.5, 0.0)), 2.0);
    }
}

////////////////////////////////////////////////////////////////
