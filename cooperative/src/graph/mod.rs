use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::EdgeId;

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
