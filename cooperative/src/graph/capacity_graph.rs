use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight, Graph};
use rust_road_router::io::{Deconstruct, Store};

pub type Capacity = u32;

pub struct CapacityGraph {
    first_out: Vec<EdgeId>,
    head: Vec<NodeId>,
    weight: Vec<Weight>,

    freeflow_weight: Vec<Weight>,
    max_capacity: Vec<Capacity>,
    used_capacity: Vec<Weight>,

    weight_function: fn(Weight, Capacity, Capacity) -> Weight,
}

pub trait ModifiableWeight<PathContainer> {
    fn increase_weights(&mut self, path: PathContainer);
    fn decrease_weights(&mut self, path: PathContainer);
    fn reset_weights(&mut self);
}

impl CapacityGraph {
    /// Create a new `CapacityGraph` from the given containers and a weight function
    pub fn new(
        first_out: Vec<EdgeId>,
        head: Vec<NodeId>,
        freeflow_weight: Vec<Weight>,
        max_capacity: Vec<Capacity>,
        weight_function: fn(Weight, Capacity, Capacity) -> Weight,
    ) -> CapacityGraph {
        assert!(first_out.len() > 0 && first_out.len() < <NodeId>::MAX as usize);
        assert!(head.len() > 0 && head.len() < <EdgeId>::MAX as usize);
        assert_eq!(first_out.first(), Some(&0));
        assert_eq!(first_out.last(), Some(&(head.len() as u32)));
        assert_eq!(freeflow_weight.len(), head.len());
        assert_eq!(max_capacity.len(), head.len());

        let weight = freeflow_weight.clone();
        let used_capacity = vec![0; max_capacity.len()];

        CapacityGraph { first_out, head, weight, freeflow_weight, max_capacity, used_capacity, weight_function }
    }

    /// Decompose the graph into its three seperate data containers
    pub fn decompose(self) -> (Vec<EdgeId>, Vec<NodeId>, Vec<Weight>) {
        (self.first_out, self.head, self.weight)
    }
}

impl Graph for CapacityGraph {
    fn num_nodes(&self) -> usize {
        self.first_out.len() - 1
    }

    fn num_arcs(&self) -> usize {
        self.head.len()
    }

    fn degree(&self, node: NodeId) -> usize {
        let node = node as usize;
        assert!(node < self.num_nodes());
        (self.first_out[node + 1] - self.first_out[node]) as usize
    }
}

impl<PathContainer> ModifiableWeight<PathContainer> for CapacityGraph
    where PathContainer: AsRef<[EdgeId]>
{
    fn increase_weights(&mut self, path: PathContainer) {
        path.as_ref().iter().cloned().for_each(|edge_id| {
            let edge_id = edge_id as usize;
            self.used_capacity[edge_id as usize] += 1;

            self.weight[edge_id] = (self.weight_function)(
                self.freeflow_weight[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id],
            );
        });
    }

    fn decrease_weights(&mut self, path: PathContainer) {
        path.as_ref().iter().cloned().for_each(|edge_id| {
            let edge_id = edge_id as usize;
            self.used_capacity[edge_id as usize] -= 1;

            self.weight[edge_id] = (self.weight_function)(
                self.freeflow_weight[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id],
            );
        });
    }

    fn reset_weights(&mut self) {
        self.used_capacity.clear();
        for i in 0..self.freeflow_weight.len() {
            self.weight[i] = self.freeflow_weight[i];
        }
    }
}

impl Deconstruct for CapacityGraph {
    fn store_each(&self, store: &dyn Fn(&str, &dyn Store) -> std::io::Result<()>) -> std::io::Result<()> {
        store("first_out", &self.first_out)?;
        store("head", &self.head)?;
        store("weights", &self.freeflow_weight)?;
        store("capacity", &self.max_capacity)?;
        Ok(())
    }
}