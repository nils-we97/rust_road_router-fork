use crate::dijkstra::corridor_elimination_tree::customized::CustomizedUpperLower;
use crate::dijkstra::corridor_elimination_tree::CorridorEliminationTreeWalk;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{NodeId, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::borrow::Borrow;
use std::cmp::min;

pub struct CorridorEliminationTreeServer<'a> {
    customized: &'a CustomizedUpperLower,
    fw_distances: TimestampedVector<(Weight, Weight)>,
    bw_distances: TimestampedVector<(Weight, Weight)>,
}

impl<'a> CorridorEliminationTreeServer<'a> {
    pub fn new(customized: &'a CustomizedUpperLower) -> Self {
        let num_nodes = customized.cch.forward_first_out().len() - 1;
        Self {
            customized,
            fw_distances: TimestampedVector::new(num_nodes, (INFINITY, INFINITY)),
            bw_distances: TimestampedVector::new(num_nodes, (INFINITY, INFINITY)),
        }
    }

    pub fn query(&mut self, from: NodeId, to: NodeId) -> Option<(Weight, Weight)> {
        // get ranks
        let from = self.customized.cch.node_order().rank(from);
        let to = self.customized.cch.node_order().rank(to);

        let (fw_graph, fw_weights) = self.customized.forward_graph();
        let (bw_graph, bw_weights) = self.customized.backward_graph();

        // initialize query datastructures
        let mut tentative_distance = (INFINITY, INFINITY);

        // initialize forward elimination tree walk
        let mut fw_walk = CorridorEliminationTreeWalk::init(
            &fw_graph,
            fw_weights,
            self.customized.cch.borrow().elimination_tree(),
            &mut self.fw_distances,
            from,
        );

        // initialize backward elimination tree walk
        let mut bw_walk = CorridorEliminationTreeWalk::init(
            &bw_graph,
            bw_weights,
            self.customized.cch.borrow().elimination_tree(),
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