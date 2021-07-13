use rust_road_router::datastr::graph::Weight;
use crate::graph::capacity_graph::Capacity;

pub fn dummy_weight_function(freeflow_weight: Weight, _: Capacity, used_capacity: Capacity) -> Weight {
    freeflow_weight + used_capacity * 30
}