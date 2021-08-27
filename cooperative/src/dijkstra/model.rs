use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight};

pub struct CapacityQueryResult {
    pub distance: Weight,
    pub path: PathResult,
}

impl CapacityQueryResult {
    pub fn new(distance: Weight, path: PathResult) -> Self {
        Self { distance, path }
    }
}

pub struct PathResult {
    pub node_path: Vec<NodeId>,
    pub edge_path: Vec<EdgeId>,
    pub departure: Vec<Timestamp>,
}

impl PathResult {
    pub fn new(node_path: Vec<NodeId>, edge_path: Vec<EdgeId>, departure: Vec<Timestamp>) -> Self {
        Self {
            node_path,
            edge_path,
            departure,
        }
    }
}
