use crate::dijkstra::elimination_tree::corridor_intervals::CorridorEliminationTreeWalk;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{EdgeId, NodeId, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::borrow::Borrow;
use std::cmp::min;

pub struct CorridorEliminationTreeServer<'a, CCH> {
    cch: &'a CCH,
    forward_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_weights: Vec<(Weight, Weight)>,
    backward_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_weights: Vec<(Weight, Weight)>,
    fw_distances: TimestampedVector<(Weight, Weight)>,
    bw_distances: TimestampedVector<(Weight, Weight)>,
}

impl<'a, CCH: CCHT> CorridorEliminationTreeServer<'a, CCH> {
    pub fn new(
        cch: &'a CCH,
        forward_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
        forward_weights: Vec<(Weight, Weight)>,
        backward_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
        backward_weights: Vec<(Weight, Weight)>,
    ) -> Self {
        let num_nodes = cch.forward_first_out().len() - 1;
        Self {
            cch,
            forward_graph,
            forward_weights,
            backward_graph,
            backward_weights,
            fw_distances: TimestampedVector::new(num_nodes, (INFINITY, INFINITY)),
            bw_distances: TimestampedVector::new(num_nodes, (INFINITY, INFINITY)),
        }
    }

    pub fn query(&mut self, from: NodeId, to: NodeId) -> Option<(Weight, Weight)> {
        // get ranks
        let from = self.cch.node_order().rank(from);
        let to = self.cch.node_order().rank(to);

        // initialize query datastructures
        let mut tentative_distance = (INFINITY, INFINITY);

        // initialize forward elimination tree walk
        let mut fw_walk = CorridorEliminationTreeWalk::init(
            &self.forward_graph,
            &self.forward_weights,
            self.cch.borrow().elimination_tree(),
            &mut self.fw_distances,
            from,
        );

        // initialize backward elimination tree walk
        let mut bw_walk = CorridorEliminationTreeWalk::init(
            &self.backward_graph,
            &self.backward_weights,
            self.cch.borrow().elimination_tree(),
            &mut self.bw_distances,
            to,
        );

        loop {
            match (fw_walk.peek(), bw_walk.peek()) {
                (Some(fw_node), Some(bw_node)) if fw_node < bw_node => {
                    // TODO consider using bound checks here as well
                    fw_walk.next();
                }
                (Some(fw_node), Some(bw_node)) if fw_node > bw_node => {
                    // TODO consider using bound checks here as well
                    bw_walk.next();
                }
                (Some(node), Some(_node)) => {
                    debug_assert_eq!(node, _node);

                    // nodes can be skipped if both the upper and lower distance is beaten by the global tentative distance
                    let fw_tent = fw_walk.tentative_distance(node);
                    if fw_tent.0 < tentative_distance.0 || fw_tent.1 < tentative_distance.1 {
                        // either the lower or the upper bounds can still be improved
                        // therefore, relax the outgoing edges of the next vertex
                        fw_walk.next();
                    } else {
                        // if both lower and upper bound distance cannot be improved anymore,
                        // the forward search can be aborted
                        fw_walk.skip_next();
                    }

                    // analogous for backward search
                    let bw_tent = bw_walk.tentative_distance(node);
                    if bw_tent.0 < tentative_distance.0 || bw_tent.1 < tentative_distance.1 {
                        bw_walk.next();
                    } else {
                        bw_walk.skip_next();
                    }

                    // update tentative distances
                    // contrary to other elimination tree queries, we don't care about the exact path,
                    // therefore the meeting node is irrelevant
                    tentative_distance = (
                        min(tentative_distance.0, fw_walk.tentative_distance(node).0 + bw_walk.tentative_distance(node).0),
                        min(tentative_distance.1, fw_walk.tentative_distance(node).1 + bw_walk.tentative_distance(node).1),
                    );
                }
                // the (Some, None) case can only happen when the nodes
                // share no common ancestors in the elimination tree
                // thus, there will be no path
                (Some(_), None) => return None,
                (None, Some(_)) => return None,
                (None, None) => break, // search space exhausted
            }
        }

        match tentative_distance {
            (INFINITY, INFINITY) => None,
            dist => Some(dist),
        }
    }
}
