pub mod td_backward_profile_potential;

use rust_road_router::datastr::graph::{NodeId, Weight};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::algo::a_star::Potential;

pub trait TDPotential {
    fn init(&mut self, target: NodeId, timestamp: Timestamp);
    fn potential(&mut self, node: NodeId, timestamp: Timestamp) -> Option<Weight>;
}

impl<T: Potential> TDPotential for T {
    fn init(&mut self, target: u32, _timestamp: u32) {
        self.init(target)
    }

    fn potential(&mut self, node: u32, _timestamp: u32) -> Option<u32> {
        self.potential(node)
    }
}