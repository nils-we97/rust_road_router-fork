pub mod capacity_graph;
pub mod td_capacity_graph;
pub mod traffic_functions;

pub trait ModifiableWeight<PathContainer> {
    fn increase_weights(&mut self, path: PathContainer);
    fn decrease_weights(&mut self, path: PathContainer);
    fn reset_weights(&mut self);
}