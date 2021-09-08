use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, Weight};

pub mod capacity_graph;
pub mod capacity_graph_traits;
pub mod edge_buckets;
pub mod speed_functions;

pub type Capacity = u32;
pub type Velocity = u32;
pub const MAX_BUCKETS: u32 = 86400000; //max timestamp

pub trait ModifiableWeight {
    fn increase_weights(&mut self, edges: &[EdgeId], departure: &[Timestamp]) -> (time::Duration, time::Duration);
    //fn decrease_weights(&mut self, path: &[(EdgeId, Timestamp)]); TODO implement as soon as needed!
    fn reset_weights(&mut self);
}

pub trait ExportableCapacity {
    fn export_capacities(&self) -> Vec<Vec<(Timestamp, Capacity)>>;
    fn update_capacities(&mut self, capacities: Vec<Vec<(Timestamp, Capacity)>>);
}

#[inline(always)]
pub fn travel_time(speed_km_h: u32, len_m: u32) -> Weight {
    100 * 36 * len_m / speed_km_h
}

#[inline(always)]
pub fn velocity(len_m: Weight, time_s: Weight) -> Velocity {
    debug_assert!(len_m > 0 && time_s > 0, "Invalid distance/time values! (must be > 0)");
    (len_m * 36) / (time_s * 10)
}
