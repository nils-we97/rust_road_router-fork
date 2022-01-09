use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight};
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct CapacityQueryResult {
    pub distance: Weight,
    pub path: PathResult,
}

impl CapacityQueryResult {
    pub fn new(distance: Weight, path: PathResult) -> Self {
        Self { distance, path }
    }
}

#[derive(Clone, Debug)]
pub struct MeasuredCapacityQueryResult {
    pub query_result: Option<CapacityQueryResult>,
    pub distance_result: DistanceMeasure,
    pub update_time: Duration,
}

#[derive(Clone, Debug)]
pub struct DistanceMeasure {
    pub distance: Option<Weight>,
    pub time_potential: Duration,
    pub time_query: Duration,
    pub num_queue_pushs: u32,
    pub num_queue_pops: u32,
    pub num_relaxed_arcs: u32,
}

#[derive(Clone, Debug)]
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
