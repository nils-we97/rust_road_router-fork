use crate::graph::capacity_graph::CapacityGraph;
use rust_road_router::datastr::graph::{EdgeIdGraph, EdgeIdT, EdgeRandomAccessGraph, Graph, Link, LinkIterable, NodeId, NodeIdT};
use std::ops::Range;

impl Graph for CapacityGraph {
    fn num_nodes(&self) -> usize {
        self.first_out().len() - 1
    }

    fn num_arcs(&self) -> usize {
        self.head().len()
    }

    fn degree(&self, node: NodeId) -> usize {
        let node = node as usize;
        assert!(node < self.num_nodes());
        (self.first_out()[node + 1] - self.first_out()[node]) as usize
    }
}

/// trait needed for `EdgeRandomAccessGraph` -> CCH potentials
impl EdgeIdGraph for CapacityGraph {
    #[rustfmt::skip]
    type IdxIter<'a> where Self: 'a = impl Iterator<Item=EdgeIdT> + 'a;

    fn edge_indices(&self, from: NodeId, to: NodeId) -> Self::IdxIter<'_> {
        self.neighbor_edge_indices(from)
            .filter(move |&edge_id| self.head()[edge_id as usize] == to)
            .map(EdgeIdT)
    }

    #[inline(always)]
    fn neighbor_edge_indices(&self, node: NodeId) -> Range<u32> {
        let node = node as usize;
        (self.first_out()[node])..(self.first_out()[node + 1])
    }

    #[inline(always)]
    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let node = node as usize;
        (self.first_out()[node] as usize)..(self.first_out()[node + 1] as usize)
    }
}

/// trait needed for CCH potentials
impl EdgeRandomAccessGraph<Link> for CapacityGraph {
    #[inline(always)]
    fn link(&self, edge_id: u32) -> Link {
        let edge_id = edge_id as usize;
        Link {
            node: self.head()[edge_id],
            weight: self.free_flow_time()[edge_id],
        }
    }
}

/// trait needed for CCH potentials
impl LinkIterable<NodeIdT> for CapacityGraph {
    type Iter<'a> = impl Iterator<Item = NodeIdT> + 'a;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head()[self.neighbor_edge_indices_usize(node)].iter().map(|&l| NodeIdT(l))
    }
}

/// trait needed for BackwardProfilePotential (creating reversed graph)
impl LinkIterable<(NodeIdT, EdgeIdT)> for CapacityGraph {
    type Iter<'a> = impl Iterator<Item = (NodeIdT, EdgeIdT)> + 'a;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head()[self.neighbor_edge_indices_usize(node)]
            .iter()
            .cloned()
            .zip(self.neighbor_edge_indices(node))
            .map(|(node, edge)| (NodeIdT(node), EdgeIdT(edge)))
    }
}

/// trait needed for CCH potentials
impl LinkIterable<Link> for CapacityGraph {
    #[allow(clippy::type_complexity)]
    type Iter<'a> = impl Iterator<Item = Link> + 'a;

    #[inline(always)]
    fn link_iter(&self, node: u32) -> Self::Iter<'_> {
        self.neighbor_edge_indices_usize(node).into_iter().map(move |idx| Link {
            node: self.head()[idx],
            weight: self.free_flow_time()[idx],
        })
    }
}
