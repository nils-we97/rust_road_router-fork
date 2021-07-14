use rust_road_router::datastr::graph::{Weight, EdgeId, NodeId};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

pub struct CapacityQueryResult<Path> {
    pub distance: Weight,
    pub path: Path
}

impl<Path> CapacityQueryResult<Path> {
    pub fn new(distance: Weight, path: Path) -> Self {
        Self { distance, path }
    }
}

pub struct PathResult {
    pub node_path: Vec<NodeId>,
    pub edge_path: Vec<EdgeId>
}

impl PathResult {
    pub fn new(node_path: Vec<NodeId>, edge_path: Vec<EdgeId>) -> Self {
        Self { node_path, edge_path }
    }
}

pub struct TDPathResult {
    pub node_path: Vec<NodeId>,
    pub edge_path: Vec<EdgeId>,
    pub departure: Vec<Timestamp>
}

impl TDPathResult {
    pub fn new(node_path: Vec<NodeId>, edge_path: Vec<EdgeId>, departure: Vec<Timestamp>) -> Self {
        Self { node_path, edge_path, departure }
    }
}
