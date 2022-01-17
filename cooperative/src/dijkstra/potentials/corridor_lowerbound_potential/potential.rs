use crate::dijkstra::potentials::cch_lower_upper::bounded_potential::BoundedLowerUpperPotential;
use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub struct CorridorLowerboundPotential<'a, CCH> {
    customized: &'a CustomizedApproximatedPeriodicTTF<CCH>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Weight>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Weight>,
    forward_potential: BoundedLowerUpperPotential<'a, CCH>,
    interval_length: u32,
    context: CorridorLowerboundPotentialContext,
}

// container for all variables which change after each query
struct CorridorLowerboundPotentialContext {
    num_pot_computations: usize,
    query_start: Timestamp,
    target_dist_bounds: Option<(Weight, Weight)>,
    backward_distances: TimestampedVector<Weight>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<Weight>>,
}

impl<'a, CCH: CCHT> CorridorLowerboundPotential<'a, CCH> {
    pub fn new(customized: &'a CustomizedApproximatedPeriodicTTF<CCH>) -> Self {
        let (forward_cch_graph, forward_cch_weights, forward_cch_bounds) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights, backward_cch_bounds) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        let forward_potential = BoundedLowerUpperPotential::new(&customized.cch, forward_cch_bounds, backward_cch_bounds);
        let interval_length = MAX_BUCKETS / customized.num_intervals;

        let context = CorridorLowerboundPotentialContext {
            num_pot_computations: 0,
            query_start: 0,
            target_dist_bounds: None,
            backward_distances: TimestampedVector::new(n),
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
        };

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_potential,
            interval_length,
            context,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.context.num_pot_computations
    }
}

impl<'a, CCH: CCHT> TDPotential for CorridorLowerboundPotential<'a, CCH> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.context.num_pot_computations = 0;
        self.context.query_start = timestamp;

        // 1. use interval query to determine the corridor at target
        self.context.target_dist_bounds = self.forward_potential.init(source, target);

        if let Some((_, target_dist_upper)) = self.context.target_dist_bounds {
            // 2. initialize custom elimination tree
            let target = self.customized.cch.node_order().rank(target);
            self.context.potentials.reset();
            self.context.backward_distances.reset();
            self.context.backward_distances[target as usize] = 0;

            let mut next = Some(target);
            while let Some(current_node) = next {
                next = self.customized.cch.elimination_tree()[current_node as usize].value();

                // additional pruning: ignore node if the distance already exceeds the target dist bounds
                if self.context.backward_distances[current_node as usize] > target_dist_upper {
                    continue;
                }

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, current_node) {
                    let edge_id = edge as usize;

                    if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(next_node) {
                        debug_assert!(target_dist_upper >= node_lower);

                        let start_idx = (((timestamp + node_lower) % MAX_BUCKETS) / self.interval_length) as usize;
                        let end_idx = (((timestamp + node_upper) % MAX_BUCKETS) / self.interval_length) as usize;

                        let mut idx = start_idx;
                        let mut edge_weight = *unsafe { self.backward_cch_weights.get_unchecked(idx * self.backward_cch_graph.num_arcs() + edge_id) };
                        while idx != end_idx {
                            idx = (idx + 1) % self.customized.num_intervals as usize;
                            edge_weight = min(edge_weight, *unsafe {
                                self.backward_cch_weights.get_unchecked(idx * self.backward_cch_graph.num_arcs() + edge_id)
                            });
                        }

                        // update distances
                        self.context.backward_distances[next_node as usize] = min(
                            self.context.backward_distances[next_node as usize],
                            self.context.backward_distances[current_node as usize] + edge_weight,
                        );
                    }
                }
            }
        }
    }

    fn potential(&mut self, node: u32, _timestamp: u32) -> Option<u32> {
        if self.context.target_dist_bounds.is_some() {
            let node = self.customized.cch.node_order().rank(node);
            let elimination_tree = self.customized.cch.elimination_tree();

            // 1. upward search until a node with existing distance to target is found
            let mut cur_node = node;
            while self.context.potentials[cur_node as usize].value().is_none() {
                self.context.num_pot_computations += 1;
                self.context.stack.push(cur_node);
                if let Some(parent) = elimination_tree[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // 2. propagate the result back to the original start node
            while let Some(current_node) = self.context.stack.pop() {
                // check if the current node is feasible, i.e. is able to reach the target within the valid corridor
                if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(current_node) {
                    let start_interval = (((self.context.query_start + node_lower) % MAX_BUCKETS) / self.interval_length) as usize;
                    let end_interval = (((self.context.query_start + node_upper) % MAX_BUCKETS) / self.interval_length) as usize;

                    for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                        // even in the forward direction, we're still performing backward linking,
                        // current edges are all starting at `current_node`
                        // -> take the same edge interval of all outgoing edges as given by the corridor
                        if let Some(next_potential) = self.context.potentials[next_node as usize].value() {
                            let mut idx = start_interval;
                            let mut edge_weight = *unsafe { self.forward_cch_weights.get_unchecked(idx * self.forward_cch_graph.num_arcs() + edge as usize) };
                            while idx != end_interval {
                                idx = (idx + 1) % self.customized.num_intervals as usize;
                                edge_weight = min(edge_weight, *unsafe {
                                    self.forward_cch_weights.get_unchecked(idx * self.forward_cch_graph.num_arcs() + edge as usize)
                                });
                            }

                            self.context.backward_distances[current_node as usize] =
                                min(self.context.backward_distances[current_node as usize], edge_weight + next_potential);
                        }
                    }
                    self.context.potentials[current_node as usize] = InRangeOption::some(self.context.backward_distances[current_node as usize]);
                } else {
                    self.context.potentials[current_node as usize] = InRangeOption::some(INFINITY);
                }
            }

            self.context.potentials[node as usize].value().filter(|&pot| pot < INFINITY)
        } else {
            None
        }
    }

    fn verify_result(&self, distance: Weight) -> bool {
        if distance < INFINITY {
            debug_assert!(self.context.target_dist_bounds.is_some());
            let upper_bound = self.context.target_dist_bounds.unwrap().1;
            debug_assert!(upper_bound >= distance, "Upper bound {} violated! Actual distance: {}", upper_bound, distance);
        }
        true
    }
}
