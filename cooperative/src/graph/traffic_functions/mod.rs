use rust_road_router::datastr::graph::{Weight, INFINITY};
use rust_road_router::datastr::graph::floating_time_dependent::FlWeight;

pub mod speed_functions;
pub mod time_functions;

fn convert_to_weight(val: f64) -> Weight {
    if val > f64::from(FlWeight::INFINITY) {
        INFINITY
    } else if val.is_sign_negative() {
        panic!("Negative weight!");
    } else {
        let ret = val.round() as Weight;
        if ret == 0 { panic!("Zero speed. This shouldn't happen!") }

        ret
    }
}