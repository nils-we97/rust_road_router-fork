use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::borrow::Borrow;
use std::cmp::min;
use std::marker::PhantomData;

pub struct CorridorEliminationTreeServer<CCH>(PhantomData<CCH>);

impl<CCH: CCHT> CorridorEliminationTreeServer<CCH> {
    pub fn query(
        cch: &CCH,
        forward_graph: &UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>,
        forward_weights: &Vec<(Weight, Weight)>,
        backward_graph: &UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>,
        backward_weights: &Vec<(Weight, Weight)>,
        fw_distances: &mut TimestampedVector<(Weight, Weight)>,
        bw_distances: &mut TimestampedVector<(Weight, Weight)>,
        from: NodeId,
        to: NodeId,
    ) -> Option<(Weight, Weight)> {
        // 1. get node ranks
        let from = cch.node_order().rank(from);
        let to = cch.node_order().rank(to);

        // 2. initialize tentative_distance
        let mut tentative_distance = (INFINITY, INFINITY);

        // initialize forward elimination tree walk
        let mut fw_walk = CorridorEliminationTreeWalk::init(forward_graph, forward_weights, cch.borrow().elimination_tree(), fw_distances, from);

        // initialize backward elimination tree walk
        let mut bw_walk = CorridorEliminationTreeWalk::init(backward_graph, backward_weights, cch.borrow().elimination_tree(), bw_distances, to);

        loop {
            match (fw_walk.peek(), bw_walk.peek()) {
                (Some(fw_node), Some(bw_node)) if fw_node < bw_node => {
                    if fw_walk.tentative_distance(fw_node).0 <= tentative_distance.1 {
                        fw_walk.next();
                    } else {
                        fw_walk.skip_next();
                    }
                }
                (Some(fw_node), Some(bw_node)) if fw_node > bw_node => {
                    if bw_walk.tentative_distance(bw_node).0 <= tentative_distance.1 {
                        bw_walk.next();
                    } else {
                        bw_walk.skip_next();
                    }
                }
                (Some(node), Some(_node)) => {
                    debug_assert_eq!(node, _node);

                    // nodes can be skipped if the lower distance already exceeds the tenative upper bound
                    if fw_walk.tentative_distance(node).0 <= tentative_distance.1 {
                        fw_walk.next();
                    } else {
                        fw_walk.skip_next();
                    }

                    // analogous for backward search
                    if bw_walk.tentative_distance(node).0 <= tentative_distance.1 {
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

#[derive(Debug)]
pub struct CorridorEliminationTreeWalk<'a> {
    graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    weights: &'a Vec<(Weight, Weight)>,
    distances: &'a mut TimestampedVector<(Weight, Weight)>,
    elimination_tree: &'a [InRangeOption<NodeId>],
    next: Option<NodeId>,
}

impl<'a> CorridorEliminationTreeWalk<'a> {
    pub fn init(
        graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
        weights: &'a Vec<(Weight, Weight)>,
        elimination_tree: &'a [InRangeOption<NodeId>],
        distances: &'a mut TimestampedVector<(Weight, Weight)>,
        from: NodeId,
    ) -> Self {
        // reset distances
        distances.reset();
        distances[from as usize] = (0, 0);

        Self {
            graph,
            weights,
            distances,
            elimination_tree,
            next: Some(from),
        }
    }

    pub fn next(&mut self) -> Option<NodeId> {
        // Examine the next node on the path to the elimination tree node
        if let Some(node) = self.next {
            self.next = self.elimination_tree[node as usize].value();

            // For each node we can reach, see if we can find a way with
            // a lower distance going through this node
            for (next_node, edge) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(self.graph, node) {
                let edge = edge.0 as usize;
                let next_node = next_node.0 as usize;

                // update tentative distances, for both lower and upper bound
                self.distances[next_node] = (
                    min(self.distances[next_node].0, self.distances[node as usize].0 + self.weights[edge].0),
                    min(self.distances[next_node].1, self.distances[node as usize].1 + self.weights[edge].1),
                );
            }

            Some(node)
        } else {
            None
        }
    }

    pub fn peek(&self) -> Option<NodeId> {
        self.next
    }

    pub fn skip_next(&mut self) {
        // Iterator::skip(n) would still call `next` and thus relax edges, we want to actually skip them
        if let Some(node) = self.next {
            self.next = self.elimination_tree[node as usize].value();
        }
    }

    pub fn tentative_distance(&self, node: NodeId) -> (Weight, Weight) {
        self.distances[node as usize]
    }
}
