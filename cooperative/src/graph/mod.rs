use rust_road_router::datastr::graph::Weight;

pub mod capacity_graph;
pub mod capacity_graph_traits;
pub mod edge_buckets;
pub mod traffic_functions;
pub mod travel_time_function;

pub type Capacity = u32;
pub type Velocity = u32;
pub const MAX_BUCKETS: u32 = 86400000; //max timestamp

#[inline(always)]
pub fn travel_time(speed_km_h: u32, len_m: u32) -> Weight {
    100 * 36 * len_m / speed_km_h
}

#[inline(always)]
pub fn velocity(len_m: Weight, time_s: Weight) -> Velocity {
    debug_assert!(len_m > 0 && time_s > 0, "Invalid distance/time values! (must be > 0)");
    (len_m * 36) / (time_s * 10)
}
