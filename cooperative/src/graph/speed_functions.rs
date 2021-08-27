use std::cmp::max;

use rust_road_router::datastr::graph::floating_time_dependent::FlWeight;
use rust_road_router::datastr::graph::{Weight, INFINITY};

use crate::graph::{Capacity, Velocity};

/// Bureau of public roads function, modification from travel time -> travel speed
pub fn bpr_speed_function(freeflow_velocity: Velocity, max_capacity: Capacity, used_capacity: Capacity) -> Weight {
    let congestion = (used_capacity as f64 / max_capacity as f64).powi(2);
    let result = freeflow_velocity as f64 / (1.0 + congestion);

    convert_to_weight(result)
}

/// clip speed in range [1, MAX_SPEED = inf], throw exception if speed is negative
#[inline(always)]
fn convert_to_weight(val: f64) -> Weight {
    if val > f64::from(FlWeight::INFINITY) {
        INFINITY
    } else if val.is_sign_negative() {
        panic!("Negative weight!");
    } else {
        max(val.round() as Weight, 1)
    }
}
