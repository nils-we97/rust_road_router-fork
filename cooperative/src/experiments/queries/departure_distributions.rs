use rand::Rng;

use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::graph::MAX_BUCKETS;
use rand_distr::Distribution;
use rand_distr::Normal;

pub trait DepartureDistribution {
    fn new() -> Self;
    fn rand<R: Rng + ?Sized>(&mut self, rng: &mut R) -> Timestamp;
}

/// constant departure distribution: all trips will start at the same time
pub struct ConstantDeparture();

impl DepartureDistribution for ConstantDeparture {
    fn new() -> Self {
        Self {}
    }

    fn rand<R: Rng + ?Sized>(&mut self, _rng: &mut R) -> Timestamp {
        0
    }
}

/// uniform departure distribution: trip departures are distributed randomly
pub struct UniformDeparture();

impl DepartureDistribution for UniformDeparture {
    fn new() -> Self {
        Self {}
    }

    fn rand<R: Rng + ?Sized>(&mut self, rng: &mut R) -> Timestamp {
        rng.gen_range(0..MAX_BUCKETS)
    }
}

/// for testing purposes only, uses a simply
pub struct NormalDeparture {
    pub distribution: Normal<f64>,
}

impl DepartureDistribution for NormalDeparture {
    fn new() -> Self {
        let distribution = Normal::new(43200.0, 7200.0).unwrap();
        Self { distribution }
    }

    fn rand<R: Rng + ?Sized>(&mut self, rng: &mut R) -> u32 {
        (self.distribution.sample(rng) as u32) * 1000
    }
}

// TODO find commonly used distributions in literature ;)

/// trip departures are following a commonly observed rush hour scheme, morning only
pub struct MorningRushHourDeparture();

/// trip departures are following a commonly observed rush hour scheme, afternoon only
pub struct AfternoonRushHourDeparture();

/// trip departures are following a commonly observed rush hour scheme
pub struct RushHourDeparture();
