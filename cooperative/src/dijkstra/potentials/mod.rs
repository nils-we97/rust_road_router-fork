use rust_road_router::algo::a_star::Potential;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{NodeId, Weight};

pub mod cch_potential_init;
pub mod td_backward_profile_potential;
pub mod td_partial_backward_profile_potential;

pub trait TDPotential {
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: Timestamp);
    fn potential(&mut self, node: NodeId, timestamp: Timestamp) -> Option<Weight>;
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

// basic conversion stuff: `TDCapacityGraph` uses integer weights, but we rely on floats here
fn convert_timestamp_u32_to_f64(ts_old: u32) -> f64 {
    let str = format!("{}.{}", ts_old / 1000, ts_old % 1000);
    str.parse::<f64>().unwrap()
}
