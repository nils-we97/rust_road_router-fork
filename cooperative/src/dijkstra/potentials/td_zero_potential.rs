use crate::dijkstra::potentials::TDPotential;
use rust_road_router::datastr::graph::{NodeId, Weight};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

pub struct TDZeroPotential();

impl TDPotential for TDZeroPotential {
    //todo change type of `Timestamp` to f64
    fn init(&mut self, _target: NodeId, _timestamp: Timestamp) {}

    fn potential(&mut self, _node: NodeId, _timestamp: Timestamp) -> Option<Weight> {
        Some(0)
    }
}