use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight, Graph, LinkIterable, Link, RandomLinkAccessGraph, NodeIdT, EdgeIdT};
use rust_road_router::io::{Deconstruct, Store};
use rust_road_router::util::SlcsIdx;
use crate::graph::ModifiableWeight;
use std::ops::Range;

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

    pub fn get_edge_id(&self, start: NodeId, end: NodeId) -> Option<EdgeId> {
        let start = start as usize;

        for i in self.first_out[start]..self.first_out[start + 1] {
            if self.head[i as usize] == end {
                return Some(i);
            }
        }

        None
    }

    pub fn weight(&self, edge_id: EdgeId) -> Weight {
        self.weight[edge_id as usize]
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

impl LinkIterable<Link> for CapacityGraph {
    #[allow(clippy::type_complexity)]
    type Iter<'a> = std::iter::Map<std::iter::Zip<std::slice::Iter<'a, NodeId>, std::slice::Iter<'a, Weight>>, fn((&NodeId, &Weight)) -> Link>;

    #[inline]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        let range = SlcsIdx(&self.first_out).range(node as usize);
        self.head[range.clone()]
            .iter()
            .zip(self.weight[range].iter())
            .map(|(&neighbor, &weight)| Link { node: neighbor, weight })
    }
}

impl LinkIterable<(NodeIdT, (Weight, EdgeIdT))> for CapacityGraph {
    #[allow(clippy::type_complexity)]
    type Iter<'a> = impl Iterator<Item=(NodeIdT, (Weight, EdgeIdT))> + 'a;

    #[inline]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        let range = SlcsIdx(&self.first_out).range(node as usize);
        self.head[range.clone()]
            .iter()
            .zip(self.weight[range.clone()].iter())
            .zip(range.into_iter())
            .map(|((&node, &weight), edge_id)| (NodeIdT(node), (weight, EdgeIdT(edge_id as EdgeId))))
    }
}

impl RandomLinkAccessGraph for CapacityGraph {
    fn link(&self, edge_id: EdgeId) -> Link {
        Link {
            node: self.head[edge_id as usize],
            weight: 0,
        }
    }

    fn edge_index(&self, from: NodeId, to: NodeId) -> Option<u32> {
        let first_out = self.first_out[from as usize];
        let range = self.neighbor_edge_indices_usize(from);
        self.head[range].iter().position(|&head| head == to).map(|pos| pos as EdgeId + first_out)
    }

    #[inline(always)]
    fn neighbor_edge_indices(&self, node: NodeId) -> Range<EdgeId> {
        (self.first_out[node as usize] as EdgeId)..(self.first_out[(node + 1) as usize] as EdgeId)
    }

    #[inline(always)]
    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let node = node as usize;
        (self.first_out[node] as usize)..(self.first_out[(node + 1)] as usize)
    }
}