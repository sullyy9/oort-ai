use std::mem::MaybeUninit;

use oort_api::prelude::Vec2;

use super::{
    draw::{self, Colour},
    kinematics::Position,
};

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct Polygon<const VERTICES: usize> {
    verticies: [Vec2; VERTICES],
}

////////////////////////////////////////////////////////////////

impl<const V: usize, T: Position> From<[T; V]> for Polygon<V> {
    fn from(verticies: [T; V]) -> Self {
        let mut poly_verts: [MaybeUninit<Vec2>; V] = unsafe { MaybeUninit::uninit().assume_init() };

        for (from, to) in std::iter::zip(&verticies, &mut poly_verts) {
            to.write(from.position());
        }

        return Self {
            verticies: poly_verts.map(|v| unsafe { v.assume_init() }),
        };
    }
}

impl<const V: usize, T: Position> From<&[T; V]> for Polygon<V> {
    fn from(verticies: &[T; V]) -> Self {
        let mut poly_verts: [MaybeUninit<Vec2>; V] = unsafe { MaybeUninit::uninit().assume_init() };

        for (from, to) in std::iter::zip(verticies, &mut poly_verts) {
            to.write(from.position());
        }

        return Self {
            verticies: poly_verts.map(|v| unsafe { v.assume_init() }),
        };
    }
}

////////////////////////////////////////////////////////////////

impl<const V: usize> Polygon<V> {
    pub fn get_verticies(&self) -> &[Vec2; V] {
        return &self.verticies;
    }
}

////////////////////////////////////////////////////////////////

impl<const V: usize> Polygon<V> {
    pub fn contains<T: Position>(&self, point: &T) -> bool {
        // Check that the polygon has an area.
        if V < 3 {
            return false;
        }

        let mut wind_number = 0;

        for points in self.verticies.windows(2) {
            let line = (&points[0], &points[1]);

            if line.0.y <= point.position().y {
                if line.1.y > point.position().y && is_left_of_line(point, line) {
                    wind_number += 1;
                }
            } else if line.1.y <= point.position().y && is_right_of_line(point, line) {
                wind_number -= 1;
            }
        }

        // Also need to check for last and first points.
        // It's safe to unwrap here since we checked at the beginning that we have at least 3
        // verticies.
        let line = (
            unsafe { self.verticies.last().unwrap_unchecked() },
            unsafe { self.verticies.first().unwrap_unchecked() },
        );

        if line.0.y <= point.position().y {
            if line.1.y > point.position().y && is_left_of_line(point, line) {
                wind_number += 1;
            }
        } else if line.1.y <= point.position().y && is_right_of_line(point, line) {
            wind_number -= 1;
        }

        return wind_number != 0;
    }
}

fn is_left_of_line<T: Position>(point: &T, line: (&Vec2, &Vec2)) -> bool {
    return ((line.1.x - line.0.x) * (point.position().y - line.0.y)
        - (point.position().x - line.0.x) * (line.1.y - line.0.y))
        > 0.0;
}

fn is_right_of_line<T: Position>(point: &T, line: (&Vec2, &Vec2)) -> bool {
    return ((line.1.x - line.0.x) * (point.position().y - line.0.y)
        - (point.position().x - line.0.x) * (line.1.y - line.0.y))
        < 0.0;
}

////////////////////////////////////////////////////////////////

impl<const V: usize> Polygon<V> {
    /// Description
    /// -----------
    /// Return the distance of the closest and furthest points of the polygon.
    ///  
    pub fn minmax_distance_to<T: Position>(&self, point: &T) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = 0.0;

        for vert in self.verticies {
            let dist = vert.distance_to(point);
            if dist < min {
                min = dist;
            }
            if dist > max {
                max = dist;
            }
        }

        return (min, max);
    }

    /// Description
    /// -----------
    /// Return the distance of the closest point of the polygon.
    ///  
    pub fn min_distance_to<T: Position>(&self, point: &T) -> f64 {
        let mut min = f64::INFINITY;

        for vert in self.verticies {
            let dist = vert.distance_to(&point.position());
            if dist < min {
                min = dist;
            }
        }

        return min;
    }

    /// Description
    /// -----------
    /// Return the distance of the closest point of the polygon.
    ///  
    pub fn max_distance_to<T: Position>(&self, point: &T) -> f64 {
        let mut max = 0.0;

        for vert in self.verticies {
            let dist = vert.distance_to(&point.position());
            if dist > max {
                max = dist;
            }
        }

        return max;
    }
}

////////////////////////////////////////////////////////////////

impl<const V: usize> Polygon<V> {
    pub fn draw(&self, colour: Colour) {
        if V < 2 {
            return;
        }

        for points in self.verticies.windows(2) {
            draw::line(&points[0], &points[1], colour);
        }

        if V >= 3 {
            draw::line(
                self.verticies.first().unwrap(),
                self.verticies.last().unwrap(),
                colour,
            );
        }
    }
}

////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use oort_api::prelude::vec2;

    use super::*;

    #[test]
    fn test_contains() {
        let poly: Polygon<4> = Polygon::from([
            vec2(2.0, 2.0),
            vec2(2.0, -2.0),
            vec2(-2.0, -2.0),
            vec2(-2.0, 2.0),
        ]);

        assert!(poly.contains(&vec2(0.0, 0.0)));
        assert!(poly.contains(&vec2(-1.0, -1.0)));
        assert!(poly.contains(&vec2(1.0, 1.0)));

        assert!(!poly.contains(&vec2(6.0, 0.0)));
        assert!(!poly.contains(&vec2(-6.0, 0.0)));
        assert!(!poly.contains(&vec2(-6.0, 5.0)));
        assert!(!poly.contains(&vec2(2.0, 0.0)));
        assert!(!poly.contains(&vec2(0.0, 2.0)));
    }
}

////////////////////////////////////////////////////////////////
