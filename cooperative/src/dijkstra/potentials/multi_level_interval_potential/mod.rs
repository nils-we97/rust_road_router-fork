pub mod label;

use crate::dijkstra::elimination_tree::corridor_intervals::server::CorridorEliminationTreeServer;
use crate::dijkstra::elimination_tree::multi_level_buckets::customized::CustomizedMultiLevels;
use crate::dijkstra::potentials::multi_level_interval_potential::label::MultiLevelBucketLabel;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;

pub struct CCHMultiLevelIntervalPotential<'a> {
    customized: &'a CustomizedMultiLevels<'a>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Vec<Weight>>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Vec<Weight>>,
    interval_query_server: CorridorEliminationTreeServer<'a, CCH>,
    num_pot_computations: usize,
    context: MultiLevelIntervalPotentialContext,
}

struct MultiLevelIntervalPotentialContext {
    stack: Vec<NodeId>,
    potentials: TimestampedVector<bool>,
    backward_distances: TimestampedVector<MultiLevelBucketLabel>,
    current_metrics: Vec<usize>,
    current_intervals: Vec<Timestamp>,
    latest_arrival_dist: Option<Weight>,
    query_start: Timestamp,
}

impl<'a> CCHMultiLevelIntervalPotential<'a> {
    pub fn new_forward(customized: &'a CustomizedMultiLevels<'a>, num_levels: usize) -> Self {
        let (forward_cch_graph, forward_cch_weights) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        // initialize lowerbound forward potential for interval query (also as fallback potential on tight corridors!)
        let pot_forward_weights = forward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();
        let pot_backward_weights = backward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();

        let interval_query_server = CorridorEliminationTreeServer::new(
            customized.cch,
            forward_cch_graph.clone(),
            pot_forward_weights,
            backward_cch_graph.clone(),
            pot_backward_weights,
        );

        let context = MultiLevelIntervalPotentialContext {
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            backward_distances: TimestampedVector::new_with_default(n, MultiLevelBucketLabel::new(num_levels + 1)),
            current_metrics: Vec::new(),
            current_intervals: Vec::new(),
            latest_arrival_dist: None,
            query_start: 0,
        };

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            interval_query_server,
            context,
            num_pot_computations: 0,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }
}

impl<'a> TDPotential for CCHMultiLevelIntervalPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.context.current_metrics.clear();
        self.context.current_intervals.clear();

        // 1. use interval query to determine the latest arrival and the relevant backward upward search space
        self.context.latest_arrival_dist = self.interval_query_server.query(source, target).map(|(_, upper)| upper);
        self.context.query_start = timestamp;

        if let Some(latest_arrival) = self.context.latest_arrival_dist {
            let latest_arrival_ts = timestamp + latest_arrival;

            // 2. determine relevant metric ids
            let tree = &self.customized.bucket_tree;
            let mut current_interval = Some(tree.find_interval(timestamp, latest_arrival).0);

            while let Some(tree_idx) = current_interval {
                self.context.current_metrics.push(tree.elements[tree_idx].metric_id);
                self.context.current_intervals.push(tree.elements[tree_idx].interval_start);

                // find first suitable child
                current_interval = None;

                for child in tree.children_range(tree_idx) {
                    if tree.elements[child].interval_start <= latest_arrival_ts && tree.elements[child].interval_end > latest_arrival_ts {
                        current_interval = Some(child);
                        break;
                    }
                }
            }

            // 3. intialize elimination tree, restrict to backward upward search space from interval query!
            let target = self.customized.cch.node_order().rank(target);
            let query_backward_distances = self.interval_query_server.backward_distances();
            self.context.potentials.reset();
            self.context.backward_distances.reset();
            self.context.backward_distances[target as usize].set_all(0);

            let mut current_node = Some(target);
            while let Some(node) = current_node {
                current_node = self.customized.cch.elimination_tree()[node as usize].value();

                // additional pruning: only relax edges if the backward distance label is set for this node!
                if query_backward_distances[node as usize].0 < INFINITY {
                    // For each node we can reach, see if we can find a way with
                    // a lower distance going through this node
                    for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, node) {
                        let edge = edge as usize;
                        let next_node = next_node as usize;

                        // update tentative distances, for both lower and upper bound
                        // careful: use correct indices, distance array only contains the relevant metrics!
                        for distance_idx in 0..self.context.current_metrics.len() {
                            let metric_idx = self.context.current_metrics[distance_idx];
                            let weight = self.context.backward_distances[node as usize][distance_idx] + self.backward_cch_weights[edge][metric_idx];

                            // updates are only relevant if the new weight is smaller than the upper bound distance to the target
                            if weight < self.context.backward_distances[next_node][distance_idx] && weight <= latest_arrival {
                                self.context.backward_distances[next_node][distance_idx] = weight;
                            }
                        }
                    }
                }
            }
        }

        self.num_pot_computations = 0;
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        // if the target isn't reachable from the source, we can abort here
        if let Some(latest_arrival_dist) = self.context.latest_arrival_dist {
            let node = self.customized.cch.node_order.rank(node);
            let timestamp = timestamp % MAX_BUCKETS;
            let elimination_tree = self.customized.cch.elimination_tree();

            // 1. upward search until a node with existing distance to target is found
            let mut cur_node = node;
            while !self.context.potentials[cur_node as usize] {
                self.num_pot_computations += 1;
                self.context.stack.push(cur_node);
                if let Some(parent) = elimination_tree[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // 2. propagate the result back to the original start node
            while let Some(current_node) = self.context.stack.pop() {
                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                    let edge_forward_weights = &self.forward_cch_weights[edge as usize];

                    /*distances
                    .iter_mut()
                    .zip(self.current_metrics.iter())
                    .zip(next_potential.iter())
                    .for_each(|((dist, &metric_id), &next_pot)| *dist = min(*dist, forward_cch_weights[metric_id] + next_pot));*/

                    for idx in 0..self.context.current_metrics.len() {
                        let metric_idx = self.context.current_metrics[idx];
                        let weight = edge_forward_weights[metric_idx] + self.context.backward_distances[next_node as usize][idx];

                        if weight < self.context.backward_distances[current_node as usize][idx] && weight <= latest_arrival_dist {
                            self.context.backward_distances[current_node as usize][idx] = weight;
                        }
                    }
                }

                self.context.potentials[current_node as usize] = true;
            }

            // 3. retrieve the value within the most-suited interval
            let pot_metric_idx = if timestamp > (self.context.query_start + latest_arrival_dist) % MAX_BUCKETS {
                // special-case treatment for changes over midnight, simply use the lowerbound pot here
                0
            } else {
                // lookup for smallest bucket that contains both `timestamp` and the upper bound arrival
                let mut val = 0;

                for i in (0..self.context.current_intervals.len()).rev() {
                    if self.context.current_intervals[i] <= timestamp {
                        val = i;
                        break;
                    }
                }
                val
            };

            Some(self.context.backward_distances[node as usize][pot_metric_idx]).filter(|&pot| pot <= latest_arrival_dist)
        } else {
            None
        }
    }
}
