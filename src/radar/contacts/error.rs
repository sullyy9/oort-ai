use oort_api::prelude::ScanResult;

////////////////////////////////////////////////////////////////

const BEARING_NOISE_FACTOR: f64 = 1e1 * (std::f64::consts::TAU / 360.0);
const DISTANCE_NOISE_FACTOR: f64 = 1e4;
const VELOCITY_NOISE_FACTOR: f64 = 1e2;

const RNG_RANGE: f64 = 4.0;

////////////////////////////////////////////////////////////////

#[derive(Clone, PartialEq, Debug)]
pub struct RadarContactError {
    pub bearing: f64,
    pub distance: f64,
    pub velocity: f64,
}

////////////////////////////////////////////////////////////////

impl From<ScanResult> for RadarContactError {
    fn from(scan: ScanResult) -> Self {
        let error_factor = 10.0_f64.powf(-scan.snr / 10.0);

        return Self {
            bearing: BEARING_NOISE_FACTOR * error_factor * RNG_RANGE,
            distance: DISTANCE_NOISE_FACTOR * error_factor * RNG_RANGE,
            velocity: VELOCITY_NOISE_FACTOR * error_factor * RNG_RANGE,
        };
    }
}

impl From<&ScanResult> for RadarContactError {
    fn from(scan: &ScanResult) -> Self {
        let error_factor = 10.0_f64.powf(-scan.snr / 10.0);

        return Self {
            bearing: BEARING_NOISE_FACTOR * error_factor * RNG_RANGE,
            distance: DISTANCE_NOISE_FACTOR * error_factor * RNG_RANGE,
            velocity: VELOCITY_NOISE_FACTOR * error_factor * RNG_RANGE,
        };
    }
}

////////////////////////////////////////////////////////////////
