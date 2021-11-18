use crate::dijkstra::elimination_tree::approximated_periodic_ttf::customized::CustomizedApproximatedPeriodicTTF;
use crate::dijkstra::potentials::bounded_lower_upper::BoundedLowerUpperPotential;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::cmp::min;

pub struct CorridorLowerboundPotential<'a> {
    customized: &'a CustomizedApproximatedPeriodicTTF<'a>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<Weight>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Vec<Weight>>,
    backward_distances: TimestampedVector<Weight>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Vec<Weight>>,
    forward_potential: BoundedLowerUpperPotential<'a, CCH>,
    num_pot_computations: usize,
    target_dist_bounds: Option<(Weight, Weight)>,
    query_start: u32,
    interval_length: u32,
}

impl<'a> CorridorLowerboundPotential<'a> {
    pub fn new(customized: &'a CustomizedApproximatedPeriodicTTF<'a>) -> Self {
        let (forward_cch_graph, forward_cch_weights, forward_cch_bounds) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights, backward_cch_bounds) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        let forward_potential = BoundedLowerUpperPotential::new(customized.cch, forward_cch_bounds.clone(), backward_cch_bounds.clone());
        let interval_length = MAX_BUCKETS / customized.num_intervals;

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_potential,
            interval_length,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            backward_distances: TimestampedVector::new(n),
            num_pot_computations: 0,
            target_dist_bounds: None,
            query_start: 0,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }
}

impl<'a> TDPotential for CorridorLowerboundPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.num_pot_computations = 0;
        self.query_start = timestamp;

        // 1. use interval query to determine the corridor at target
        self.target_dist_bounds = self.forward_potential.init(source, target);

        if let Some((target_dist_lower, target_dist_upper)) = self.target_dist_bounds {
            // 2. initialize custom elimination tree
            let target = self.customized.cch.node_order().rank(target);
            self.potentials.reset();
            self.backward_distances.reset();
            self.backward_distances[target as usize] = 0;

            let mut next = Some(target);
            while let Some(current_node) = next {
                next = self.customized.cch.elimination_tree()[current_node as usize].value();

                // additional pruning: ignore node if the distance already exceeds the target dist bounds
                if self.backward_distances[current_node as usize] > target_dist_upper {
                    continue;
                }

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, current_node) {
                    let edge_id = edge as usize;

                    if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(next_node) {
                        if target_dist_upper >= node_lower {
                            // collect start & end interval of current node
                            let start = if target_dist_lower > node_upper {
                                timestamp + target_dist_lower - node_upper
                            } else {
                                timestamp
                            };

                            let start_idx = ((start % MAX_BUCKETS) / self.interval_length) as usize;
                            let end_idx = (((timestamp + target_dist_upper - node_lower) % MAX_BUCKETS) / self.interval_length) as usize;

                            let edge_weight = if start_idx <= end_idx {
                                self.backward_cch_weights[edge_id][start_idx..=end_idx].iter().min().cloned().unwrap()
                            } else {
                                min(
                                    self.backward_cch_weights[edge_id][start_idx..].iter().min().cloned().unwrap(),
                                    self.backward_cch_weights[edge_id][..=end_idx].iter().min().cloned().unwrap(),
                                )
                            };

                            // update distances
                            self.backward_distances[next_node as usize] = min(
                                self.backward_distances[next_node as usize],
                                self.backward_distances[current_node as usize] + edge_weight,
                            );
                        }
                    }
                }
            }
        }
    }

    fn potential(&mut self, node: u32, _timestamp: u32) -> Option<u32> {
        if let Some((target_dist_lower, target_dist_upper)) = self.target_dist_bounds {
            let node = self.customized.cch.node_order.rank(node);
            let elimination_tree = self.customized.cch.elimination_tree();

            // 1. upward search until a node with existing distance to target is found
            let mut cur_node = node;
            while self.potentials[cur_node as usize] == INFINITY {
                self.num_pot_computations += 1;
                self.stack.push(cur_node);
                if let Some(parent) = elimination_tree[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // 2. propagate the result back to the original start node
            while let Some(current_node) = self.stack.pop() {
                // check if the current node is feasible, i.e. is able to reach the target within the valid corridor
                if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(current_node) {
                    let start = if target_dist_lower > node_upper {
                        self.query_start + target_dist_lower - node_upper
                    } else {
                        self.query_start
                    };

                    let start_interval = ((start % MAX_BUCKETS) / self.interval_length) as usize;
                    let end_interval = (((self.query_start + target_dist_upper - node_lower) % MAX_BUCKETS) / self.interval_length) as usize;

                    for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                        // even in the forward direction, we're still performing backward linking,
                        // current edges are all starting at `current_node`
                        // -> take the same edge interval of all outgoing edges as given by the corridor

                        // cover edge case: intervals span over midnight
                        let edge_interval_min = if start_interval <= end_interval {
                            self.forward_cch_weights[edge as usize][start_interval..=end_interval]
                                .iter()
                                .min()
                                .cloned()
                                .unwrap()
                        } else {
                            min(
                                self.forward_cch_weights[edge as usize][start_interval..].iter().min().cloned().unwrap(),
                                self.forward_cch_weights[edge as usize][..=end_interval].iter().min().cloned().unwrap(),
                            )
                        };
                        self.backward_distances[current_node as usize] = min(
                            self.backward_distances[current_node as usize],
                            edge_interval_min + self.potentials[next_node as usize],
                        );
                    }
                    self.potentials[current_node as usize] = self.backward_distances[current_node as usize];
                }
            }

            Some(self.potentials[node as usize]).filter(|&pot| pot < INFINITY)
        } else {
            None
        }
    }
}
