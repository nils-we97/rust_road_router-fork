use rust_road_router::datastr::graph::Weight;

use crate::graph::capacity_graph::Capacity;
use crate::graph::traffic_functions::convert_to_weight;
use crate::graph::td_capacity_graph::Velocity;

pub fn bpr_speed_function(
    freeflow_velocity: Velocity,
    max_capacity: Capacity,
    used_capacity: Capacity
) -> Weight {
    let congestion = (used_capacity as f64 / max_capacity as f64).powi(2);
    let result = freeflow_velocity as f64 / (1.0 + congestion);

    convert_to_weight(result)
}