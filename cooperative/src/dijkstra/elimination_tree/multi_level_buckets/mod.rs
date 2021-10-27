use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub mod bucket_tree;
pub mod customized;

#[derive(Debug)]
pub struct MultiLevelEliminationTreeWalk<'a> {
    graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    weights: &'a Vec<Vec<Weight>>,
    used_metrics: &'a Vec<usize>,
    distances: &'a mut TimestampedVector<Vec<Weight>>,
    elimination_tree: &'a [InRangeOption<NodeId>],
    next: Option<NodeId>,
}

impl<'a> MultiLevelEliminationTreeWalk<'a> {
    pub fn init(
        graph: &'a UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
        weights: &'a Vec<Vec<Weight>>,
        used_metrics: &'a Vec<usize>,
        elimination_tree: &'a [InRangeOption<NodeId>],
        distances: &'a mut TimestampedVector<Vec<Weight>>,
        from: NodeId,
    ) -> Self {
        // reset distances
        distances.reset();
        distances[from as usize] = vec![0; used_metrics.len()];

        Self {
            graph,
            weights,
            used_metrics,
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
                // careful: use correct indices, distance array only contains the relevant metrics!
                self.used_metrics.iter().enumerate().for_each(|(distance_idx, &metric_idx)| {
                    self.distances[next_node][distance_idx] = min(
                        self.distances[next_node][distance_idx],
                        self.distances[node as usize][distance_idx] + self.weights[edge][metric_idx],
                    );
                });
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

    pub fn tentative_distance(&self, node: NodeId) -> &Vec<Weight> {
        &self.distances[node as usize]
    }
}
