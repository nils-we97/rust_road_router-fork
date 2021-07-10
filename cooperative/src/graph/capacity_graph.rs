use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight, FirstOutGraph, Graph};

pub type Capacity = u32;

pub struct CapacityWeight {
    weight: Weight,
    freeflow_weight: Weight,
    max_capacity: Capacity,
    used_capacity: Capacity
}

pub trait ModifiableWeight {
    fn increase_weights(&self, path: Vec<EdgeId>);
    fn decrease_weights(&self, path: Vec<EdgeId>);
    fn reset_weights(&self);
}

pub type CapacityGraph = FirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<CapacityWeight>>;

impl ModifiableWeight for CapacityGraph {
    fn increase_weights(&self, path: Vec<EdgeId>) {
        for edge_id in path.iter().cloned() {
            todo!()
        }
    }

    fn decrease_weights(&self, path: Vec<EdgeId>) {
        for edge_id in path.iter().cloned() {
            todo!()
        }
    }

    fn reset_weights(&self) {
        todo!()
    }
}