use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::EdgeId;

pub mod capacity_graph;
pub mod speed_functions;

pub type Capacity = u32;
pub type Velocity = u32;
pub const MAX_BUCKETS: u32 = 86400000; //max timestamp

pub trait ModifiableWeight {
    fn increase_weights(&mut self, path: &[(EdgeId, Timestamp)]);
    fn decrease_weights(&mut self, path: &[(EdgeId, Timestamp)]);
    fn reset_weights(&mut self);
}
