use std::ops::Range;

use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, EdgeIdT, Graph, Link, LinkIterable, NodeId, NodeIdT, Weight, EdgeRandomAccessGraph};
use rust_road_router::io::{Deconstruct, Store};
use rust_road_router::util::SlcsIdx;

use crate::graph::ModifiableWeight;

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
    pub fn decompose(self) -> (Vec<EdgeId>, Vec<NodeId>, Vec<Weight>, Vec<Capacity>) {
        (self.first_out, self.head, self.weight, self.max_capacity)
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

    pub fn head(&self, edge_id: EdgeId) -> NodeId { self.head[edge_id as usize] }

    pub fn weight(&self, edge_id: EdgeId) -> Weight {
        self.weight[edge_id as usize]
    }

    pub fn capacity(&self, edge_id: EdgeId) -> Capacity { self.max_capacity[edge_id as usize] }
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
        store("weight", &self.freeflow_weight)?;
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

impl LinkIterable<NodeIdT> for CapacityGraph {
    type Iter<'a> = impl Iterator<Item=NodeIdT> + 'a;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head[self.neighbor_edge_indices_usize(node)].iter().map(|&l| NodeIdT(l))
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

impl EdgeIdGraph for CapacityGraph {
    #[rustfmt::skip]
    type IdxIter<'a> where Self: 'a = impl Iterator<Item=EdgeIdT> + 'a;

    fn edge_indices(&self, from: NodeId, to: NodeId) -> Self::IdxIter<'_> {
        self
            .neighbor_edge_indices(from)
            .filter(move |&edge_id| self.head(edge_id) == to)
            .map(EdgeIdT)
    }

    fn neighbor_edge_indices(&self, node: NodeId) -> Range<EdgeId> {
        let node = node as usize;
        (self.first_out[node])..(self.first_out[node + 1])
    }

    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let node = node as usize;
        (self.first_out[node] as usize)..(self.first_out[node + 1] as usize)
    }
}

impl EdgeRandomAccessGraph<Link> for CapacityGraph {
    fn link(&self, edge_id: u32) -> Link {
        Link {
            node: self.head(edge_id),
            weight: self.weight(edge_id),
        }
    }
}