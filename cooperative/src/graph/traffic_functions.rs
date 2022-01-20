use std::cmp::max;

use rust_road_router::datastr::graph::floating_time_dependent::FlWeight;
use rust_road_router::datastr::graph::{Weight, INFINITY};

use crate::graph::Capacity;

/// Bureau of public roads function, modification from travel time -> travel speed
#[derive(Clone, Debug)]
pub struct BPRTrafficFunction {
    alpha: f64,
    beta: i32,
}

impl BPRTrafficFunction {
    pub fn default() -> Self {
        Self { alpha: 1.0, beta: 2 }
    }

    pub fn new(alpha: f64, beta: i32) -> Self {
        assert!(beta > 0, "Factor beta must be chosen greater than zero!");
        Self { alpha, beta }
    }

    pub fn travel_time(&self, free_flow_time: Weight, max_capacity: Capacity, used_capacity: Capacity) -> Weight {
        if free_flow_time == INFINITY || max_capacity == 0 {
            INFINITY
        } else {
            let result = free_flow_time as f64 * (1.0 + self.alpha * (used_capacity as f64 / max_capacity as f64).powi(self.beta));

            // return value should be in range [1, INFINITY]
            assert!(result < f64::from(FlWeight::INFINITY), "Travel time must not be infinity!");
            max(result.round() as Weight, 1)
        }
    }

    pub fn speed(&self, free_flow_speed: Weight, max_capacity: Capacity, used_capacity: Capacity) -> Weight {
        if free_flow_speed == 0 || max_capacity == 0 {
            1
        } else {
            let result = free_flow_speed as f64 / (1.0 + self.alpha * (used_capacity as f64 / max_capacity as f64).powi(self.beta));

            // speeds are restricted to [1, free_flow_speed]
            max(result.round() as Weight, 1)
        }
    }
}
