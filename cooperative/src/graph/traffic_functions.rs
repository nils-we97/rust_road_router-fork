use std::cmp::max;

use rust_road_router::datastr::graph::floating_time_dependent::FlWeight;
use rust_road_router::datastr::graph::{Weight, INFINITY};

use crate::graph::Capacity;

const BPR_ALPHA: f64 = 0.15;
const BPR_BETA: i32 = 4;

/// Bureau of public roads function, modification from travel time -> travel speed
pub fn bpr_traffic_function(freeflow_time: Weight, max_capacity: Capacity, used_capacity: Capacity) -> Weight {
    let result = freeflow_time as f64 * (1.0 + BPR_ALPHA * (used_capacity as f64 / max_capacity as f64).powi(BPR_BETA));

    // return value should be in range [1, INFINITY]
    if result >= f64::from(FlWeight::INFINITY) {
        INFINITY
    } else {
        max(result.round() as Weight, 1)
    }
}