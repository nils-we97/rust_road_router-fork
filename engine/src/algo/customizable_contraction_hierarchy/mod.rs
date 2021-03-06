//! Implementation of Customizable Contraction Hierarchies.

use super::*;
use crate::{
    datastr::node_order::NodeOrder,
    io::*,
    report::{benchmark::*, block_reporting},
    util::{in_range_option::InRangeOption, *},
};
use std::{cmp::Ordering, ops::Range};

mod contraction;
use contraction::*;
mod customization;
pub use customization::ftd as ftd_cch;
pub use customization::{customize, customize_directed, customize_perfect};
pub mod separator_decomposition;
use separator_decomposition::*;
mod reorder;
use crate::util::in_range_option::Sentinel;
pub use reorder::*;
pub mod query;

/// Execute first phase, that is metric independent preprocessing.
pub fn contract<Graph: LinkIterable<NodeIdT> + EdgeIdGraph>(graph: &Graph, node_order: NodeOrder) -> CCH {
    CCH::new(ContractionGraph::new(graph, node_order).contract())
}

/// A struct containing all metric independent preprocessing data of CCHs.
/// This includes on top of the chordal supergraph (the "contracted" graph),
/// several other structures like the elimination tree, a mapping from cch edge ids to original edge ids and the inverted graph.
pub struct CCH {
    pub first_out: Vec<EdgeId>,
    pub head: Vec<NodeId>,
    pub tail: Vec<NodeId>,
    pub node_order: NodeOrder,
    pub forward_cch_edge_to_orig_arc: Vecs<EdgeIdT>,
    pub backward_cch_edge_to_orig_arc: Vecs<EdgeIdT>,
    pub elimination_tree: Vec<InRangeOption<NodeId>>,
    pub inverted: ReversedGraphWithEdgeIds,
}

impl Deconstruct for CCH {
    fn store_each(&self, store: &dyn Fn(&str, &dyn Store) -> std::io::Result<()>) -> std::io::Result<()> {
        store("cch_first_out", &self.first_out)?;
        store("cch_head", &self.head)?;
        Ok(())
    }
}

pub struct CCHReconstrctor<'g, Graph>(pub &'g Graph);

impl<'g, Graph: EdgeIdGraph> ReconstructPrepared<CCH> for CCHReconstrctor<'g, Graph> {
    fn reconstruct_with(self, loader: Loader) -> std::io::Result<CCH> {
        let node_order = NodeOrder::reconstruct_from(&loader.path())?;
        let head: Vec<NodeId> = loader.load("cch_head")?;
        let cch_graph = UnweightedOwnedGraph::new(loader.load("cch_first_out")?, head);
        assert_eq!(cch_graph.num_nodes(), self.0.num_nodes());
        Ok(CCH::new_from(self.0, node_order, cch_graph))
    }
}

impl CCH {
    pub fn fix_order_and_build(graph: &(impl LinkIterable<NodeIdT> + EdgeIdGraph), order: NodeOrder) -> Self {
        let cch = {
            let _blocked = block_reporting();
            contract(graph, order)
        };
        let order = CCHReordering {
            cch: &cch,
            latitude: &[],
            longitude: &[],
        }
        .reorder_for_seperator_based_customization();
        contract(graph, order)
    }

    fn new<Graph: EdgeIdGraph>(contracted_graph: ContractedGraph<Graph>) -> CCH {
        let (cch, order, orig) = contracted_graph.decompose();
        Self::new_from(orig, order, cch)
    }

    // this method creates all the other structures from the contracted graph
    fn new_from<Graph: EdgeIdGraph>(original_graph: &Graph, node_order: NodeOrder, contracted_graph: UnweightedOwnedGraph) -> Self {
        let elimination_tree = Self::build_elimination_tree(&contracted_graph);
        let n = contracted_graph.num_nodes() as NodeId;
        let m = contracted_graph.num_arcs();
        let mut tail = vec![0; m];

        let order = &node_order;
        let forward_cch_edge_to_orig_arc = Vecs::from_iters((0..n).flat_map(|node| {
            LinkIterable::<NodeIdT>::link_iter(&contracted_graph, node)
                .map(move |NodeIdT(neighbor)| original_graph.edge_indices(order.node(node), order.node(neighbor)))
        }));
        let backward_cch_edge_to_orig_arc = Vecs::from_iters((0..n).flat_map(|node| {
            LinkIterable::<NodeIdT>::link_iter(&contracted_graph, node)
                .map(move |NodeIdT(neighbor)| original_graph.edge_indices(order.node(neighbor), order.node(node)))
        }));

        for node in 0..n {
            tail[contracted_graph.neighbor_edge_indices_usize(node)]
                .iter_mut()
                .for_each(|tail| *tail = node);
        }

        let inverted = ReversedGraphWithEdgeIds::reversed(&contracted_graph);
        let (first_out, head) = contracted_graph.decompose();

        CCH {
            first_out,
            head,
            node_order,
            forward_cch_edge_to_orig_arc,
            backward_cch_edge_to_orig_arc,
            elimination_tree,
            tail,
            inverted,
        }
    }

    /// Reconstruct the separators of the nested dissection order.
    pub fn separators(&self) -> SeparatorTree {
        SeparatorTree::new(&self.elimination_tree)
    }

    fn build_elimination_tree(graph: &UnweightedOwnedGraph) -> Vec<InRangeOption<NodeId>> {
        (0..graph.num_nodes())
            .map(|node_id| LinkIterable::<NodeIdT>::link_iter(graph, node_id as NodeId).map(|NodeIdT(n)| n).min())
            .map(InRangeOption::new)
            .collect()
    }

    /// Get the tail node for an edge id
    pub fn edge_id_to_tail(&self, edge_id: EdgeId) -> NodeId {
        self.tail[edge_id as usize]
    }

    /// Get chordal supergraph `first_out` as slice
    pub fn first_out(&self) -> &[EdgeId] {
        &self.first_out
    }

    /// Get chordal supergraph `head` as slice
    pub fn head(&self) -> &[NodeId] {
        &self.head
    }

    pub fn mem_size(&self) -> usize {
        // node data: first_out, node_order (2 x 4 Bytes), elimination tree (4 Bytes)
        let node_size = std::mem::size_of_val(&*self.first_out) + self.num_nodes() * 12;

        // edge data: head, tail
        let edge_size = std::mem::size_of_val(&*self.head) + std::mem::size_of_val(&*self.tail);

        // shortcuts
        let forward_shortcut_size = self.forward_cch_edge_to_orig_arc.iter().map(|slc| std::mem::size_of_val(&*slc)).sum::<usize>();

        let backward_shortcut_size = self.forward_cch_edge_to_orig_arc.iter().map(|slc| std::mem::size_of_val(&*slc)).sum::<usize>();

        // reverse graph: first_out, head, edge_ref
        let rev_graph_size = (self.num_nodes() + 1) * 4 + self.num_arcs() * 8;

        node_size + edge_size + forward_shortcut_size + backward_shortcut_size + rev_graph_size
    }

    #[inline]
    pub fn neighbor_edge_indices(&self, node: NodeId) -> Range<EdgeId> {
        (self.first_out[node as usize])..(self.first_out[(node + 1) as usize])
    }

    #[inline]
    pub fn edge_indices_range(&self, nodes: Range<NodeId>) -> Range<EdgeId> {
        (self.first_out[nodes.start as usize] as EdgeId)..(self.first_out[nodes.end as usize] as EdgeId)
    }

    #[inline]
    pub fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let range = self.neighbor_edge_indices(node);
        Range {
            start: range.start as usize,
            end: range.end as usize,
        }
    }

    #[inline]
    pub fn edge_indices_range_usize(&self, nodes: Range<NodeId>) -> Range<usize> {
        let range = self.edge_indices_range(nodes);
        Range {
            start: range.start as usize,
            end: range.end as usize,
        }
    }

    #[inline]
    pub fn neighbor_iter(&self, node: NodeId) -> std::iter::Cloned<std::slice::Iter<NodeId>> {
        let range = self.neighbor_edge_indices_usize(node);
        self.head[range].iter().cloned()
    }

    /// Transform into a directed CCH which is more efficient
    /// for turn expanded graphs because many edges can be removed.
    pub fn into_directed_cch(self) -> DirectedCCH {
        // identify arcs which are always infinity and can be removed
        let customized = customization::always_infinity(&self);
        let forward = customized.forward_graph();
        let backward = customized.backward_graph();

        let mut forward_first_out = Vec::with_capacity(self.first_out.len());
        forward_first_out.push(0);
        let mut forward_head = Vec::with_capacity(self.head.len());

        let mut backward_first_out = Vec::with_capacity(self.first_out.len());
        backward_first_out.push(0);
        let mut backward_head = Vec::with_capacity(self.head.len());

        let forward_cch_edge_to_orig_arc = Vecs::from_iters(
            self.forward_cch_edge_to_orig_arc
                .iter()
                .zip(forward.weight().iter())
                .filter(|(_, w)| **w < INFINITY)
                .map(|(slc, _)| slc.iter().copied()),
        );
        let backward_cch_edge_to_orig_arc = Vecs::from_iters(
            self.backward_cch_edge_to_orig_arc
                .iter()
                .zip(backward.weight().iter())
                .filter(|(_, w)| **w < INFINITY)
                .map(|(slc, _)| slc.iter().copied()),
        );

        for node in 0..self.num_nodes() as NodeId {
            forward_head.extend(LinkIterable::<Link>::link_iter(&forward, node).filter(|l| l.weight < INFINITY).map(|l| l.node));
            backward_head.extend(LinkIterable::<Link>::link_iter(&backward, node).filter(|l| l.weight < INFINITY).map(|l| l.node));
            forward_first_out.push(forward_head.len() as EdgeId);
            backward_first_out.push(backward_head.len() as EdgeId);
        }

        let forward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&forward_first_out[..], &forward_head[..]));
        let backward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&backward_first_out[..], &backward_head[..]));

        DirectedCCH {
            forward_first_out,
            forward_head,
            backward_first_out,
            backward_head,
            node_order: self.node_order,
            forward_cch_edge_to_orig_arc,
            backward_cch_edge_to_orig_arc,
            elimination_tree: self.elimination_tree,
            forward_inverted,
            backward_inverted,
        }
    }
}

impl Graph for CCH {
    fn num_arcs(&self) -> usize {
        self.head.len()
    }

    fn num_nodes(&self) -> usize {
        self.first_out.len() - 1
    }

    fn degree(&self, node: NodeId) -> usize {
        let node = node as usize;
        (self.first_out[node + 1] - self.first_out[node]) as usize
    }
}

/// Trait for directed and undirected CCHs
pub trait CCHT {
    fn forward_first_out(&self) -> &[EdgeId];
    fn backward_first_out(&self) -> &[EdgeId];
    fn forward_head(&self) -> &[NodeId];
    fn backward_head(&self) -> &[NodeId];
    fn forward_inverted(&self) -> &ReversedGraphWithEdgeIds;
    fn backward_inverted(&self) -> &ReversedGraphWithEdgeIds;

    /// Get elimination tree (actually forest).
    /// The tree is represented as a slice of length `n`.
    /// The entry with index `x` contains the parent node in the tree of node `x`.
    /// If there is no parent, `x` is a root node.
    fn elimination_tree(&self) -> &[InRangeOption<NodeId>];

    /// Borrow node order
    fn node_order(&self) -> &NodeOrder;

    /// Check for a node pair and a weight if there is a corresponding lower triangle.
    /// If so, return the id of the middle node and the weights of both lower edges.
    fn unpack_arc(&self, from: NodeId, to: NodeId, weight: Weight, upward: &[Weight], downward: &[Weight]) -> Option<(NodeId, Weight, Weight)> {
        unpack_arc(from, to, weight, upward, downward, self.forward_inverted(), self.backward_inverted())
    }
}

pub fn unpack_arc(
    from: NodeId,
    to: NodeId,
    weight: Weight,
    upward: &[Weight],
    downward: &[Weight],
    forward_inverted: &ReversedGraphWithEdgeIds,
    backward_inverted: &ReversedGraphWithEdgeIds,
) -> Option<(NodeId, Weight, Weight)> {
    // `inverted` contains the downward neighbors sorted ascending.
    // We do a coordinated linear sweep over both neighborhoods.
    // Whenever we find a common neighbor, we have a lower triangle.
    let mut current_iter = backward_inverted.link_iter(from).peekable();
    let mut other_iter = forward_inverted.link_iter(to).peekable();

    debug_assert_eq!(upward.len(), forward_inverted.num_arcs());
    debug_assert_eq!(downward.len(), backward_inverted.num_arcs());

    while let (
        Some(&(NodeIdT(lower_from_first), Reversed(EdgeIdT(edge_from_first_id)))),
        Some(&(NodeIdT(lower_from_second), Reversed(EdgeIdT(edge_from_second_id)))),
    ) = (current_iter.peek(), other_iter.peek())
    {
        match lower_from_first.cmp(&lower_from_second) {
            Ordering::Less => current_iter.next(),
            Ordering::Greater => other_iter.next(),
            Ordering::Equal => {
                if downward[edge_from_first_id as usize] + upward[edge_from_second_id as usize] == weight {
                    return Some((lower_from_first, downward[edge_from_first_id as usize], upward[edge_from_second_id as usize]));
                }

                current_iter.next();
                other_iter.next()
            }
        };
    }

    None
}

/// A struct containing all metric independent preprocessing data of CCHs.
/// This includes on top of the chordal supergraph (the "contracted" graph),
/// several other structures like the elimination tree, a mapping from cch edge ids to original edge ids and the inverted graph.
impl CCHT for CCH {
    fn forward_first_out(&self) -> &[EdgeId] {
        &self.first_out[..]
    }
    fn backward_first_out(&self) -> &[EdgeId] {
        &self.first_out[..]
    }
    fn forward_head(&self) -> &[NodeId] {
        &self.head[..]
    }
    fn backward_head(&self) -> &[NodeId] {
        &self.head[..]
    }
    fn forward_inverted(&self) -> &ReversedGraphWithEdgeIds {
        &self.inverted
    }
    fn backward_inverted(&self) -> &ReversedGraphWithEdgeIds {
        &self.inverted
    }

    fn node_order(&self) -> &NodeOrder {
        &self.node_order
    }

    fn elimination_tree(&self) -> &[InRangeOption<NodeId>] {
        &self.elimination_tree[..]
    }
}

/// A struct containing the results of the second preprocessing phase.
#[derive(Debug)]
pub struct Customized<CCH, CCHRef> {
    cch: CCHRef,
    upward: Vec<Weight>,
    downward: Vec<Weight>,
    _phantom: std::marker::PhantomData<CCH>,
}

impl<C: CCHT, CCHRef: std::borrow::Borrow<C>> Customized<C, CCHRef> {
    pub fn new(cch: CCHRef, upward: Vec<Weight>, downward: Vec<Weight>) -> Self {
        Customized {
            cch,
            upward,
            downward,
            _phantom: Default::default(),
        }
    }

    pub fn forward_graph(&self) -> FirstOutGraph<&[EdgeId], &[NodeId], &[Weight]> {
        FirstOutGraph::new(self.cch.borrow().forward_first_out(), self.cch.borrow().forward_head(), &self.upward)
    }

    pub fn backward_graph(&self) -> FirstOutGraph<&[EdgeId], &[NodeId], &[Weight]> {
        FirstOutGraph::new(self.cch.borrow().backward_first_out(), self.cch.borrow().backward_head(), &self.downward)
    }

    pub fn cch(&self) -> &C {
        self.cch.borrow()
    }
}

pub struct DirectedCCH {
    forward_first_out: Vec<EdgeId>,
    forward_head: Vec<NodeId>,
    backward_first_out: Vec<EdgeId>,
    backward_head: Vec<NodeId>,
    node_order: NodeOrder,
    forward_cch_edge_to_orig_arc: Vecs<EdgeIdT>,
    backward_cch_edge_to_orig_arc: Vecs<EdgeIdT>,
    elimination_tree: Vec<InRangeOption<NodeId>>,
    forward_inverted: ReversedGraphWithEdgeIds,
    backward_inverted: ReversedGraphWithEdgeIds,
}

impl DirectedCCH {
    pub fn new(
        forward_first_out: Vec<EdgeId>,
        forward_head: Vec<NodeId>,
        backward_first_out: Vec<EdgeId>,
        backward_head: Vec<NodeId>,
        node_order: NodeOrder,
        forward_cch_edge_to_orig_arc: Vec<Vec<EdgeIdT>>,
        backward_cch_edge_to_orig_arc: Vec<Vec<EdgeIdT>>,
        elimination_tree: Vec<InRangeOption<NodeId>>,
        forward_inverted: ReversedGraphWithEdgeIds,
        backward_inverted: ReversedGraphWithEdgeIds,
    ) -> Self {
        let forward_cch_edge_to_orig_arc = Vecs::from_iters(forward_cch_edge_to_orig_arc.iter().map(|v| v.iter().cloned()));

        let backward_cch_edge_to_orig_arc = Vecs::from_iters(backward_cch_edge_to_orig_arc.iter().map(|v| v.iter().cloned()));

        Self {
            forward_first_out,
            forward_head,
            backward_first_out,
            backward_head,
            node_order,
            forward_cch_edge_to_orig_arc,
            backward_cch_edge_to_orig_arc,
            elimination_tree,
            forward_inverted,
            backward_inverted,
        }
    }

    pub fn num_nodes(&self) -> usize {
        self.forward_first_out.len() - 1
    }

    fn forward(&self) -> Slcs<EdgeId, NodeId> {
        Slcs(&self.forward_first_out, &self.forward_head)
    }

    fn backward(&self) -> Slcs<EdgeId, NodeId> {
        Slcs(&self.forward_first_out, &self.forward_head)
    }

    pub fn mem_size(&self) -> usize {
        // node data: first_out (forward/backward), node_order (2 x 4 Bytes), elimination tree (4 Bytes)
        let node_size = std::mem::size_of_val(&*self.forward_first_out) + std::mem::size_of_val(&*self.backward_first_out) + self.num_nodes() * 12;

        // edge data: head, tail
        let edge_size = std::mem::size_of_val(&*self.forward_head) + std::mem::size_of_val(&*self.backward_head);

        // shortcuts
        let forward_shortcut_size = self.forward_cch_edge_to_orig_arc.iter().map(|a| std::mem::size_of_val(&*a)).sum::<usize>();
        let backward_shortcut_size = self.backward_cch_edge_to_orig_arc.iter().map(|a| std::mem::size_of_val(&*a)).sum::<usize>();

        // reverse graph: first_out, head, edge_ref
        let forward_rev_graph_size = (self.num_nodes() + 1) * 4 + self.forward_head.len() * 8;
        let backward_rev_graph_size = (self.num_nodes() + 1) * 4 + self.backward_head.len() * 8;

        node_size + edge_size + forward_shortcut_size + backward_shortcut_size + forward_rev_graph_size + backward_rev_graph_size
    }

    /// Reconstruct the separators of the nested dissection order.
    pub fn separators(&self) -> SeparatorTree {
        SeparatorTree::new(&self.elimination_tree)
    }
}

impl CCHT for DirectedCCH {
    fn forward_first_out(&self) -> &[EdgeId] {
        &self.forward_first_out[..]
    }
    fn backward_first_out(&self) -> &[EdgeId] {
        &self.backward_first_out[..]
    }
    fn forward_head(&self) -> &[NodeId] {
        &self.forward_head[..]
    }
    fn backward_head(&self) -> &[NodeId] {
        &self.backward_head[..]
    }
    fn forward_inverted(&self) -> &ReversedGraphWithEdgeIds {
        &self.forward_inverted
    }
    fn backward_inverted(&self) -> &ReversedGraphWithEdgeIds {
        &self.backward_inverted
    }

    fn node_order(&self) -> &NodeOrder {
        &self.node_order
    }

    fn elimination_tree(&self) -> &[InRangeOption<NodeId>] {
        &self.elimination_tree[..]
    }
}

impl Deconstruct for DirectedCCH {
    fn store_each(&self, store: &dyn Fn(&str, &dyn Store) -> std::io::Result<()>) -> std::io::Result<()> {
        store("forward_first_out", &self.forward_first_out)?;
        store("forward_head", &self.forward_head)?;
        store("backward_first_out", &self.backward_first_out)?;
        store("backward_head", &self.backward_head)?;
        store("ranks", &self.node_order.ranks().to_vec())?;

        let elimination_tree = self
            .elimination_tree
            .iter()
            .map(|val| val.value().unwrap_or(u32::SENTINEL))
            .collect::<Vec<u32>>();
        store("elimination_tree", &elimination_tree)?;

        // build prefix sum for forward edge conversion
        let mut fw_prefix_sum = vec![0];
        self.forward_cch_edge_to_orig_arc
            .iter()
            .for_each(|v| fw_prefix_sum.push(*fw_prefix_sum.last().unwrap() + v.len() as u32));
        let forward_cch_edge_to_orig_arc = self.forward_cch_edge_to_orig_arc.iter().flatten().map(|&EdgeIdT(v)| v).collect::<Vec<u32>>();
        store("forward_cch_edge_to_orig_arc", &forward_cch_edge_to_orig_arc)?;
        store("forward_cch_edge_to_orig_arc_prefix_sum", &fw_prefix_sum)?;

        // build prefix sum for backward edge conversion
        let mut bw_prefix_sum = vec![0];
        self.backward_cch_edge_to_orig_arc
            .iter()
            .for_each(|v| bw_prefix_sum.push(*bw_prefix_sum.last().unwrap() + v.len() as u32));
        let backward_cch_edge_to_orig_arc = self.backward_cch_edge_to_orig_arc.iter().flatten().map(|&EdgeIdT(v)| v).collect::<Vec<u32>>();
        store("backward_cch_edge_to_orig_arc", &backward_cch_edge_to_orig_arc)?;
        store("backward_cch_edge_to_orig_arc_prefix_sum", &bw_prefix_sum)?;

        Ok(())
    }
}

impl Reconstruct for DirectedCCH {
    fn reconstruct_with(loader: Loader) -> std::io::Result<Self> {
        let forward_first_out: Vec<EdgeId> = loader.load("forward_first_out")?;
        let forward_head: Vec<NodeId> = loader.load("forward_head")?;
        let backward_first_out: Vec<EdgeId> = loader.load("backward_first_out")?;
        let backward_head: Vec<EdgeId> = loader.load("backward_head")?;
        let node_order = loader.load("ranks").map(NodeOrder::from_ranks)?;
        let elimination_tree: Vec<u32> = loader.load("elimination_tree")?;

        let elimination_tree = elimination_tree
            .iter()
            .map(|&val| {
                if val == u32::SENTINEL {
                    InRangeOption::NONE
                } else {
                    InRangeOption::some(val)
                }
            })
            .collect::<Vec<InRangeOption<u32>>>();

        let forward_cch_edge_to_orig_arc_raw: Vec<u32> = loader.load("forward_cch_edge_to_orig_arc")?;
        let fw_prefix_sum: Vec<u32> = loader.load("forward_cch_edge_to_orig_arc_prefix_sum")?;

        let forward_cch_edge_to_orig_arc = fw_prefix_sum
            .windows(2)
            .map(|a| {
                let lower_inclusive = a[0] as usize;
                let upper_exclusive = a[1] as usize;

                forward_cch_edge_to_orig_arc_raw[lower_inclusive..upper_exclusive]
                    .iter()
                    .cloned()
                    .map(EdgeIdT)
                    .collect::<Vec<EdgeIdT>>()
            })
            .collect::<Vec<Vec<EdgeIdT>>>();

        let backward_cch_edge_to_orig_arc_raw: Vec<u32> = loader.load("backward_cch_edge_to_orig_arc")?;
        let bw_prefix_sum: Vec<u32> = loader.load("backward_cch_edge_to_orig_arc_prefix_sum")?;

        let backward_cch_edge_to_orig_arc = bw_prefix_sum
            .windows(2)
            .map(|a| {
                let lower_inclusive = a[0] as usize;
                let upper_exclusive = a[1] as usize;

                backward_cch_edge_to_orig_arc_raw[lower_inclusive..upper_exclusive]
                    .iter()
                    .cloned()
                    .map(EdgeIdT)
                    .collect::<Vec<EdgeIdT>>()
            })
            .collect::<Vec<Vec<EdgeIdT>>>();

        let forward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&forward_first_out[..], &forward_head[..]));
        let backward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&backward_first_out[..], &backward_head[..]));

        Ok(DirectedCCH::new(
            forward_first_out,
            forward_head,
            backward_first_out,
            backward_head,
            node_order,
            forward_cch_edge_to_orig_arc,
            backward_cch_edge_to_orig_arc,
            elimination_tree,
            forward_inverted,
            backward_inverted,
        ))
    }
}
