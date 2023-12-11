use oort_api::prelude::*;

use crate::draw::{self, Colour};
use crate::math::kinematics::Position;

use super::{
    point::{AsPoint, Point},
    shape::{EllipticalShape, Shape},
    vector::{AsVector, Vector},
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct Ellipse {
    centre: Point,
    orientation: f64,
    width: f64,
    height: f64,
}

////////////////////////////////////////////////////////////////

impl Position for Ellipse {
    fn position(&self) -> Vec2 {
        return self.centre.clone().into();
    }
}

////////////////////////////////////////////////////////////////

impl Ellipse {
    /// Description
    /// -----------
    /// Create a new ellipse.
    /// Since +x axis is heading 0, height is size along the x axis.
    ///
    pub fn new<T: AsPoint>(centre: &T, orientation: f64, width: f64, height: f64) -> Self {
        // Make them always be W I D E B O I S. That way we know which axis to iterate over when
        // drawing.
        return if width > height {
            Self {
                centre: centre.as_point(),
                orientation: orientation + std::f64::consts::FRAC_PI_2,
                width: height,
                height: width,
            }
        } else {
            return Self {
                centre: centre.as_point(),
                orientation,
                width,
                height,
            };
        };
    }
}

////////////////////////////////////////////////////////////////

impl Shape for Ellipse {
    fn translate<T: AsVector>(&mut self, vector: &T) {
        let vector = vector.as_vector();
        self.centre = self.centre.translated_by(&vector);
    }

    fn contains<T: AsPoint>(&self, point: &T) -> bool {
        let point = point.as_point();
        let vector = point
            .vector_to(&Point::from(self.position()))
            .rotated(-self.orientation);

        let a = self.height / 2.0;
        let b = self.width / 2.0;

        let xpart = (vector.x()).powi(2) / a.powi(2);
        let ypart = (vector.y()).powi(2) / b.powi(2);
        return (xpart + ypart) <= 1.0;
    }

    fn min_distance_to<T: AsPoint>(&self, point: &T) -> f64 {
        let point = point.as_point();

        let centre_distance = point.distance_to(&self.centre);
        let radius = self.radius(point.bearing_to(&self.centre));

        return (centre_distance - radius).abs();
    }

    fn max_distance_to<T: AsPoint>(&self, point: &T) -> f64 {
        let point = point.as_point();

        let centre_distance = point.distance_to(&self.centre);
        let radius = self.radius(point.bearing_to(&self.centre));

        return centre_distance + radius;
    }

    fn minmax_distance_to<T: AsPoint>(&self, point: &T) -> (f64, f64) {
        let point = point.as_point();

        let centre_distance = point.distance_to(&self.centre);
        let radius = self.radius(point.bearing_to(&self.centre));

        return ((centre_distance - radius).abs(), centre_distance + radius);
    }

    fn draw(&self, colour: Colour) {
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
            points.push(Vector::new(x, y(x)));
        }
        points.push(Vector::new(a, y(a)));

        // So far we only have half of the shape but the other half is just a reflection over the
        // x axis.
        for i in (1..(points.len() - 1)).rev() {
            let p = &points[i];
            points.push(Vector::new(p.x(), -p.y()));
        }

        // So far the shape has been constructed without it's correct orrientation and around the
        // origin. Rotate and move it into the correct position.
        let points: Vec<Point> = points
            .iter()
            .map(|p| p.rotated(self.orientation))
            .map(|p| self.centre.translated_by(&p))
            .collect();

        // Draw the shape.
        for pair in points.windows(2) {
            let (p1, p2) = (&pair[0], &pair[1]);
            draw::line(&Vec2::from(p1.clone()), &Vec2::from(p2.clone()), colour);
        }
        draw::line(
            &Vec2::from(points.first().unwrap().clone()),
            &Vec2::from(points.last().unwrap().clone()),
            colour,
        );
    }
}

////////////////////////////////////////////////////////////////

impl EllipticalShape for Ellipse {
    fn radius(&self, angle: f64) -> f64 {
        let angle = angle - self.orientation;
        let semi_major_axis = self.height / 2.0;
        let semi_minor_axis = self.width / 2.0;

        let radius = (semi_major_axis * semi_minor_axis)
            / f64::sqrt(
                (semi_major_axis.powi(2) * f64::sin(angle).powi(2))
                    + (semi_minor_axis.powi(2) * f64::cos(angle).powi(2)),
            );

        return radius;
    }

    fn expand(&mut self, value: f64) {
        self.width += value * 2.0;
        self.height += value * 2.0;
    }
}

////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    use super::*;

    #[test]
    fn test_radius() {
        let height = 10.0;
        let width = 5.0;
        let ellipse = Ellipse::new(&vec2(0.0, 0.0), 0.0, width, height);

        assert_eq!(ellipse.radius(0.0), height / 2.0);
        assert_eq!(ellipse.radius(FRAC_PI_2), width / 2.0);
        assert_eq!(ellipse.radius(PI), height / 2.0);
        assert_eq!(ellipse.radius(PI + FRAC_PI_2), width / 2.0);

        let height = 5.0;
        let width = 10.0;
        let ellipse = Ellipse::new(&vec2(0.0, 0.0), FRAC_PI_4, width, height);

        assert_eq!(ellipse.radius(FRAC_PI_4), height / 2.0);
        assert_eq!(ellipse.radius(FRAC_PI_2 + FRAC_PI_4), width / 2.0);
        assert_eq!(ellipse.radius(PI + FRAC_PI_4), height / 2.0);
        assert_eq!(ellipse.radius(-FRAC_PI_4), width / 2.0);
    }

    #[test]
    fn test_contains() {
        let ellipse = Ellipse::new(&Point::new(0.0, 0.0), 0.0, 5.0, 10.0);

        assert!(ellipse.contains(&Point::new(4.0, 0.0)));
        assert!(ellipse.contains(&Point::new(0.0, 2.0)));

        assert!(!ellipse.contains(&Point::new(0.0, 3.0)));
        assert!(!ellipse.contains(&Point::new(0.0, -6.0)));
        assert!(!ellipse.contains(&Point::new(5.0, -6.0)));
        assert!(!ellipse.contains(&Point::new(0.0, 6.0)));
        assert!(!ellipse.contains(&Point::new(6.0, 0.0)));

        let ellipse = Ellipse::new(
            &Point::new(0.0, 0.0),
            std::f64::consts::FRAC_PI_4,
            5.0,
            10.0,
        );
        assert!(ellipse.contains(&Point::new(3.0, 3.0)));
        assert!(!ellipse.contains(&Point::new(-3.0, 3.0)));
    }

    #[test]
    fn test_max_distance_to() {
        let ellipse = Ellipse::new(&Point::new(0.0, 5.0), 0.0, 5.0, 10.0);

        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 0.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 10.0)), 7.5);
        assert_eq!(ellipse.max_distance_to(&Point::new(5.0, 5.0)), 10.0);
        assert_eq!(ellipse.max_distance_to(&Point::new(-5.0, 5.0)), 10.0);

        // Questionable behaviour when point is at the centre of the ellipse.
        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 5.0)), 5.0);
        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 6.0)), 3.5);

        // 45 degree angle with tip touching the origin.
        let height = f64::sqrt(5.0_f64.powi(2) + 5.0_f64.powi(2)) * 2.0;
        let ellipse = Ellipse::new(
            &Point::new(5.0, 5.0),
            std::f64::consts::FRAC_PI_4,
            50.0,
            height,
        );

        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 0.0)), height);
        assert_eq!(ellipse.max_distance_to(&Point::new(10.0, 10.0)), height);

        // Above but mirrored.
        let angle = std::f64::consts::FRAC_PI_4 + std::f64::consts::FRAC_PI_2;
        let ellipse = Ellipse::new(&Point::new(5.0, 5.0), angle, 50.0, height);
        assert_eq!(ellipse.max_distance_to(&Point::new(0.0, 10.0)), height);
    }

    #[test]
    fn test_min_distance_to() {
        let ellipse = Ellipse::new(
            &Point::new(5.0, 0.0),
            std::f64::consts::FRAC_PI_2,
            5.0,
            10.0,
        );

        assert_eq!(ellipse.min_distance_to(&Point::new(0.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&Point::new(10.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&Point::new(5.0, 5.0)), 0.0);
        assert_eq!(ellipse.min_distance_to(&Point::new(5.0, -5.0)), 0.0);

        assert_eq!(ellipse.min_distance_to(&Point::new(5.0, 0.0)), 2.5);
        assert_eq!(ellipse.min_distance_to(&Point::new(5.5, 0.0)), 2.0);
    }
}

////////////////////////////////////////////////////////////////
