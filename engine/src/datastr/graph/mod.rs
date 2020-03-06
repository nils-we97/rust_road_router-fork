//! Something with graphs 🤷‍♂️
//!
//! Several traits and structs for working with graphs.

use crate::datastr::node_order::NodeOrder;
use std::ops::Range;

pub mod first_out_graph;
pub mod floating_time_dependent;
pub mod link_id_to_tail_mapper;
pub mod time_dependent;

pub use self::first_out_graph::{FirstOutGraph, OwnedGraph};

/// Node ids are 32bit unsigned ints
pub type NodeId = u32;
/// Edge ids are 32bit unsigned ints
pub type EdgeId = u32;
/// Basic weights are 32bit unsigned ints
pub type Weight = u32;
/// A sufficiently large infinity constant.
/// Set to `u32::MAX / 2` so that `INFINITY + x` for `x <= INFINITY` does not overflow.
pub const INFINITY: Weight = std::u32::MAX / 2;

/// Simple struct for weighted links.
/// No behaviour, just a pure data struct.
#[derive(Debug, Copy, Clone)]
pub struct Link {
    pub node: NodeId,
    pub weight: Weight,
}

/// Base trait for graphs.
/// Interesting behaviour will be added through subtraits.
pub trait Graph {
    fn num_nodes(&self) -> usize;
    fn num_arcs(&self) -> usize;
    fn degree(&self, node: NodeId) -> usize;
}

/// Trait for graph data structures which allow iterating over outgoing links of a node.
pub trait LinkIterGraph<'a>: Graph {
    /// Type of the outgoing neighbor iterator.
    /// Currently fixed to Links, but this could be generalized relatively easily
    /// (though it would be difficult to specify bounds on the Link type, because currently we can't
    /// extract associated types from higher ranked trait bounds. Concrete types work fine though).
    /// Whats less easy is the lifetime bound, which currently has to come from a lifetime param of the trait.
    /// This can be fixed once we have GATs https://github.com/rust-lang/rfcs/pull/1598
    type Iter: Iterator<Item = Link> + 'a;

    /// Get a iterator over the outgoing links of the given node.
    fn neighbor_iter(&'a self, node: NodeId) -> Self::Iter;

    /// Create a new graph with all edges reversed
    fn reverse(&'a self) -> OwnedGraph {
        // vector of adjacency lists for the reverse graph
        let mut reversed: Vec<Vec<Link>> = (0..self.num_nodes()).map(|_| Vec::<Link>::new()).collect();

        // iterate over all edges and insert them in the reversed structure
        for node in 0..(self.num_nodes() as NodeId) {
            for Link { node: neighbor, weight } in self.neighbor_iter(node) {
                reversed[neighbor as usize].push(Link { node, weight });
            }
        }

        OwnedGraph::from_adjancecy_lists(reversed)
    }

    /// Split the graph in an upward graph of outgoing edges and a
    /// downward graph of incoming edges based on the node order passed.
    fn ch_split(&'a self, order: &NodeOrder) -> (OwnedGraph, OwnedGraph) {
        let mut up: Vec<Vec<Link>> = (0..self.num_nodes()).map(|_| Vec::<Link>::new()).collect();
        let mut down: Vec<Vec<Link>> = (0..self.num_nodes()).map(|_| Vec::<Link>::new()).collect();

        // iterate over all edges and insert them in the reversed structure
        for node in 0..(self.num_nodes() as NodeId) {
            for Link { node: neighbor, weight } in self.neighbor_iter(node) {
                if order.rank(node) < order.rank(neighbor) {
                    up[node as usize].push(Link { node: neighbor, weight });
                } else {
                    down[neighbor as usize].push(Link { node, weight });
                }
            }
        }

        (OwnedGraph::from_adjancecy_lists(up), OwnedGraph::from_adjancecy_lists(down))
    }
}

/// Trait for graph data structures which allow iterating over outgoing links of a node and modifying the weights of those links.
pub trait MutWeightLinkIterGraph<'a>: Graph {
    /// Type of the outgoing neighbor iterator.
    type Iter: Iterator<Item = (&'a NodeId, &'a mut Weight)> + 'a;
    /// Get a iterator with mutable weights over the outgoing links of the given node.
    fn mut_weight_link_iter(&'a mut self, node: NodeId) -> Self::Iter;
}

/// Utility function for preprocessing graphs with parallel edges.
/// Will ensure, that all parallel edges have the lowest weight.
/// Currently used to preprocess OSM graphs before CCH customization
/// because the respecting will not necessarily use the parallel edge with the lowest weight.
pub fn unify_parallel_edges<G: for<'a> MutWeightLinkIterGraph<'a>>(graph: &mut G) {
    let mut weight_cache = vec![INFINITY; graph.num_nodes()];
    for node in 0..graph.num_nodes() {
        for (&node, weight) in graph.mut_weight_link_iter(node as NodeId) {
            weight_cache[node as usize] = std::cmp::min(weight_cache[node as usize], *weight);
        }
        for (&node, weight) in graph.mut_weight_link_iter(node as NodeId) {
            *weight = weight_cache[node as usize];
        }
        for (&node, _weight) in graph.mut_weight_link_iter(node as NodeId) {
            weight_cache[node as usize] = INFINITY;
        }
    }
}

/// Trait for graph types which allow random access to links based on edge ids.
pub trait RandomLinkAccessGraph: Graph {
    /// Get the link with the given id.
    fn link(&self, edge_id: EdgeId) -> Link;
    /// Find the id of the edge from `from` to `to` if it exists.
    fn edge_index(&self, from: NodeId, to: NodeId) -> Option<EdgeId>;
    /// Get the range of edge ids which make up the outgoing edges of `node`
    fn neighbor_edge_indices(&self, node: NodeId) -> Range<EdgeId>;

    /// Get the range of edge ids which make up the outgoing edges of `node` as a `Range<usize>`
    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let range = self.neighbor_edge_indices(node);
        Range {
            start: range.start as usize,
            end: range.end as usize,
        }
    }
}
