use oort_api::prelude::*;

use super::{
    draw::{self, Colour},
    math::{
        geometry::Ellipse,
        kinematics::{Acceleration, AngularVelocity, Heading, Position, Velocity},
    },
};

////////////////////////////////////////////////////////////////

pub struct ContactDrawer {
    acceleration: Vec2,
}

////////////////////////////////////////////////////////////////

impl Position for ContactDrawer {
    fn position(&self) -> Vec2 {
        return position();
    }
}

////////////////////////////////////////////////////////////////

impl Velocity for ContactDrawer {
    fn velocity(&self) -> Vec2 {
        return velocity();
    }
}

////////////////////////////////////////////////////////////////

impl Acceleration for ContactDrawer {
    fn acceleration(&self) -> Vec2 {
        return self.acceleration;
    }
}

////////////////////////////////////////////////////////////////

impl Heading for ContactDrawer {
    fn heading(&self) -> f64 {
        return heading();
    }
}

////////////////////////////////////////////////////////////////

impl AngularVelocity for ContactDrawer {
    fn angular_velocity(&self) -> f64 {
        return angular_velocity();
    }
}

////////////////////////////////////////////////////////////////

const BEARING_NOISE_FACTOR: f64 = 1e1 * (std::f64::consts::TAU / 360.0);
const DISTANCE_NOISE_FACTOR: f64 = 1e4;

impl ContactDrawer {
    pub fn new() -> Self {
        debug!("spawn fighter team 0 position (50, 0) heading 0");
        debug!("spawn missile team 1 position (3000, 3000) heading 0");

        return Self {
            acceleration: vec2(0.0, 0.0),
        };
    }

    pub fn tick(&mut self) {
        set_radar_heading(std::f64::consts::FRAC_PI_4);

        draw_line(
            vec2(0.0, 0.0),
            vec2(100.0, 0.0).rotate(std::f64::consts::FRAC_PI_4),
            Colour::Red as u32,
        );

        if let Some(scan) = scan() {
            let distance = self.distance_to(&scan.position);
            let bearing = scan.position.bearing_to(self);

            // Generate the distance error range given a minimum probability.
            // Each distance point in the range will have a probability.
            // Need to re-calculate the max bearing error at each distance point given the minimum
            // allowed probability.

            // Oort uses the StandardNormal distribution, which is equivilent to the Normal
            // distribution with: mean = 0.0 & standard deviation = 1.0
            // probability = (value - mean) / standard deviation.
            let mean = 0.0;
            let stddev = 1.0;

            let min_probability = 0.001;

            let max_rng = max_value(min_probability, mean, stddev);

            ////////////////////////////////

            let error_factor = 10.0_f64.powf(-scan.snr / 10.0);
            let bearing_error = BEARING_NOISE_FACTOR * error_factor * max_rng;
            let distance_error = DISTANCE_NOISE_FACTOR * error_factor * max_rng;

            ////////////////////////////////

            // Get list of points in the distance error range and their probabilities.
            let mut distance_errors = Vec::new();
            let mut distance_rng = 0.0;
            let step = max_rng / 20.0;

            while distance_rng <= (max_rng + step) {
                let prob = probability_of(distance_rng, mean, stddev);
                let dist = distance_rng * DISTANCE_NOISE_FACTOR * error_factor;
                distance_errors.push((dist, prob));

                distance_rng += step;
            }

            ////////////////////////////////

            // Transform the list of distance error points into distance and bearing error points.
            let mut errors = Vec::new();
            for (dist, prob) in distance_errors {
                let bearing_prob = min_probability / prob;
                let bearing_rng = max_value(bearing_prob, mean, stddev);

                let bear = bearing_rng * BEARING_NOISE_FACTOR * error_factor;

                errors.push((dist, bear));
            }

            ////////////////////////////////

            let mut points = Vec::new();
            for (dist_error, bear_error) in errors.iter() {
                let dist = distance + dist_error;
                let bear = bearing + bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }

            for (dist_error, bear_error) in errors.iter() {
                let dist = distance - dist_error;
                let bear = bearing + bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }

            for (dist_error, bear_error) in errors.iter() {
                let dist = distance - dist_error;
                let bear = bearing - bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }

            for (dist_error, bear_error) in errors.iter() {
                let dist = distance + dist_error;
                let bear = bearing - bear_error;
                let point = self.position() + vec2(dist, 0.0).rotate(bear);
                points.push(point);
            }

            ////////////////////////////////

            debug!("points: {}", points.len());
            for points in points.windows(2) {
                let (p1, p2) = (points[0], points[1]);

                draw::line(&p1, &p2, Colour::Green);
            }

            ////////////////////////////////

            // Now draw an ellipse using distance and bearing error. Hopefully it should match or
            // at least be close to what we already have.
            let width = f64::atan(bearing_error) * distance * 2.0;
            let height = distance_error * 2.0;

            let ellipse = Ellipse::new(&scan.position, bearing, width, height);
            ellipse.draw(Colour::Purple);

            let ellipse =
                Ellipse::new(&self.position(), std::f64::consts::FRAC_PI_4, width, height);
            ellipse.draw(Colour::Purple);

            ////////////////////////////////
        }
    }
}

fn probability_of(value: f64, mean: f64, stddev: f64) -> f64 {
    let num = f64::exp(-0.5 * f64::powf((value - mean) / stddev, 2.0));
    let den = stddev * f64::sqrt(2.0 * std::f64::consts::PI);

    return num / den;
}

fn max_value(prob: f64, mean: f64, stddev: f64) -> f64 {
    return (stddev
        * f64::sqrt(-2.0 * f64::ln(prob * stddev * f64::sqrt(2.0 * std::f64::consts::PI))))
        + mean;
}

////////////////////////////////////////////////////////////////
