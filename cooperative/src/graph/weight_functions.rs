use rust_road_router::datastr::graph::{Weight, INFINITY};
use crate::graph::capacity_graph::Capacity;
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight};

pub fn dummy_weight_function(freeflow_weight: Weight, _: Capacity, used_capacity: Capacity) -> Weight {
    freeflow_weight + used_capacity * 30
}

pub fn bpr_traffic_function(
    freeflow_weight: Weight,
    max_capacity: Capacity,
    used_capacity: Capacity
) -> Weight {
    let congestion = (used_capacity as f64 / max_capacity as f64).powi(2);
    let result = freeflow_weight as f64 * (1.0 + congestion);

    convert_to_weight(result)
}

/// clip value to allowed range [0, Weight::INFINITY]
fn convert_to_weight(val: f64) -> Weight {
    if val > f64::from(FlWeight::INFINITY) {
        INFINITY
    } else if val.is_sign_negative() {
        0
    } else {
        val.round() as Weight
    }
}