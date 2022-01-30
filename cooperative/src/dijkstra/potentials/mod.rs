use rust_road_router::algo::a_star::Potential;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{NodeId, Weight};

pub mod cch_lower_upper;
pub mod cch_parallelization_util;
pub mod corridor_lowerbound_potential;
pub mod init_cch_potential;
pub mod multi_metric_potential;

pub trait TDPotential {
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: Timestamp);
    fn potential(&mut self, node: NodeId, timestamp: Timestamp) -> Option<Weight>;

    fn verify_result(&self, _distance: Weight) -> bool {
        true
    }
}

impl<T: Potential> TDPotential for T {
    fn init(&mut self, _source: u32, target: u32, _timestamp: u32) {
        self.init(target)
    }

    fn potential(&mut self, node: u32, _timestamp: u32) -> Option<u32> {
        self.potential(node)
    }
}

// additional helper functions

/// basic conversion: `CapacityGraph` uses integer weights, but we rely on floats here
pub fn convert_timestamp_u32_to_f64(ts_old: u32) -> f64 {
    (ts_old as f64) / 1000.0
}

pub fn convert_timestamp_f64_to_u32(ts_old: f64) -> u32 {
    // avoid floating point errors -> round by 4 decimal places before conversion
    // (by construction, there won't be more than 3 decimal places)
    let ts_old = (ts_old * 10000.0).round() / 10000.0;
    (1000.0 * ts_old) as u32
}
