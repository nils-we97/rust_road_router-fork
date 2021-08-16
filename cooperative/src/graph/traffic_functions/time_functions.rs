use crate::graph::capacity_graph::Capacity;
use crate::graph::traffic_functions::convert_to_weight;
use rust_road_router::datastr::graph::Weight;

pub fn dummy_weight_function(freeflow_weight: Weight, _: Capacity, used_capacity: Capacity) -> Weight {
    freeflow_weight + used_capacity * 30
}

pub fn bpr_traffic_function(freeflow_weight: Weight, max_capacity: Capacity, used_capacity: Capacity) -> Weight {
    let congestion = (used_capacity as f64 / max_capacity as f64).powi(2);
    let result = freeflow_weight as f64 * (1.0 + congestion);

    convert_to_weight(result)
}
