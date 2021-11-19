use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub mod customized;
pub mod customized_catchup;
pub mod interval_minima;

#[derive(Debug)]
pub struct CorridorLowerboundEliminationTreeWalk<'a> {
    graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    distances: &'a mut TimestampedVector<Weight>,
    elimination_tree: &'a [InRangeOption<NodeId>],
    next: Option<NodeId>,
}

impl<'a> CorridorLowerboundEliminationTreeWalk<'a> {
    pub fn init(
        graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
        elimination_tree: &'a [InRangeOption<NodeId>],
        distances: &'a mut TimestampedVector<Weight>,
        from: NodeId,
    ) -> Self {
        // reset distances
        distances.reset();
        distances[from as usize] = 0;

        Self {
            graph,
            distances,
            elimination_tree,
            next: Some(from),
        }
    }

    pub fn next(&mut self, outgoing_links: &[(NodeIdT, Weight)]) {
        debug_assert!(self.next.is_some(), "Make sure to use peek() before calling this method!");
        let node = self.next.unwrap();

        self.next = self.elimination_tree[node as usize].value();

        for &(next_node, edge_weight) in outgoing_links {
            let next_node = next_node.0 as usize;
            self.distances[next_node] = min(self.distances[next_node], self.distances[node as usize] + edge_weight);
        }
    }

    pub fn peek(&self) -> Option<(NodeId, Vec<(NodeIdT, EdgeIdT)>)> {
        self.next.map(|node| {
            (
                node,
                LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(self.graph, node).collect::<Vec<(NodeIdT, EdgeIdT)>>(),
            )
        })
    }
}
