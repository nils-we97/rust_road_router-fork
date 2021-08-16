use rust_road_router::datastr::graph::floating_time_dependent::FlWeight;
use rust_road_router::datastr::graph::{Weight, INFINITY};
use std::cmp::max;

pub mod speed_functions;
pub mod time_functions;

/// clip speed in range [1, MAX_SPEED = inf], throw exception if speed is negative
fn convert_to_weight(val: f64) -> Weight {
    if val > f64::from(FlWeight::INFINITY) {
        INFINITY
    } else if val.is_sign_negative() {
        panic!("Negative weight!");
    } else {
        max(val.round() as Weight, 1)
    }
}
