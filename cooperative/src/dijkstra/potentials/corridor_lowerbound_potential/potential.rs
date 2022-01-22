use crate::dijkstra::potentials::cch_lower_upper::bounded_potential::BoundedLowerUpperPotential;
use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{DirectedCCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::borrow::Borrow;
use std::cmp::min;

// container for all variables which change after each query
#[derive(Debug, Clone)]
pub struct CorridorLowerboundPotentialContext {
    num_pot_computations: usize,
    query_start: Timestamp,
    target_dist_bounds: Option<(Weight, Weight)>,
    backward_distances: TimestampedVector<Weight>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<Weight>>,
}

impl CorridorLowerboundPotentialContext {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            num_pot_computations: 0,
            query_start: 0,
            target_dist_bounds: None,
            backward_distances: TimestampedVector::new(num_nodes),
            stack: Vec::new(),
            potentials: TimestampedVector::new(num_nodes),
        }
    }
}

pub struct CorridorLowerboundPotential<'a> {
    cch: &'a DirectedCCH,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Weight>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Weight>,
    forward_potential: BoundedLowerUpperPotential<'a, DirectedCCH>,
    interval_length: u32,
    num_intervals: u32,
    context: &'a mut CorridorLowerboundPotentialContext,
}

impl<'a> CorridorLowerboundPotential<'a> {
    pub fn prepare(customized: &'a mut CustomizedCorridorLowerbound) -> Self {
        let forward_cch_graph = UnweightedFirstOutGraph::new(customized.cch.borrow().forward_first_out(), customized.cch.borrow().forward_head());
        let backward_cch_graph = UnweightedFirstOutGraph::new(customized.cch.borrow().backward_first_out(), customized.cch.borrow().backward_head());

        let forward_potential = BoundedLowerUpperPotential::prepare(
            &customized.cch,
            &customized.upward_bounds,
            &customized.downward_bounds,
            &mut customized.corridor_context,
        );

        Self {
            cch: &customized.cch,
            forward_cch_graph,
            forward_cch_weights: &customized.upward_intervals,
            backward_cch_graph,
            backward_cch_weights: &customized.downward_intervals,
            forward_potential,
            interval_length: MAX_BUCKETS / customized.num_intervals,
            num_intervals: customized.num_intervals,
            context: &mut customized.potential_context,
        }
    }

    pub fn prepare_ptv(customized: &'a mut CustomizedCorridorLowerbound) -> Self {
        // todo for ptv graphs, we use the same cch structure
        Self::prepare(customized)
    }

    /*pub fn new(customized: &'a CustomizedCorridorLowerbound) -> Self {
        let (forward_cch_graph, forward_cch_weights, forward_cch_bounds) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights, backward_cch_bounds) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        let forward_potential = BoundedLowerUpperPotential::new(&customized.cch, forward_cch_bounds.clone(), backward_cch_bounds.clone());
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
    }*/

    /*pub fn new_with_separate_bounds(customized_ttf: &'a CustomizedCorridorLowerbound<DirectedCCH>, customized_bounds: &'a CustomizedLowerUpper) -> Self {
        // init main pot structs
        let (forward_cch_graph, forward_cch_weights, _) = customized_ttf.forward_graph();
        let (backward_cch_graph, backward_cch_weights, _) = customized_ttf.backward_graph();

        let n = customized_ttf.cch.forward_first_out().len() - 1;
        let interval_length = MAX_BUCKETS / customized_ttf.num_intervals;
        let context = CorridorLowerboundPotentialContext {
            num_pot_computations: 0,
            query_start: 0,
            target_dist_bounds: None,
            backward_distances: TimestampedVector::new(n),
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
        };

        // init corridor pot structs
        let forward_potential = BoundedLowerUpperPotential::new(&customized_bounds.cch, customized_bounds.upward.clone(), customized_bounds.downward.clone());

        Self {
            customized: customized_ttf,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_potential,
            interval_length,
            context,
        }
    }

    pub fn refresh_bounds(&mut self, customized_bounds: &'a CustomizedLowerUpper) {
        self.forward_potential = BoundedLowerUpperPotential::new(&customized_bounds.cch, customized_bounds.upward.clone(), customized_bounds.downward.clone());
    }*/

    pub fn num_pot_computations(&self) -> usize {
        self.context.num_pot_computations
    }
}

impl<'a> TDPotential for CorridorLowerboundPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.context.num_pot_computations = 0;
        self.context.query_start = timestamp;

        // 1. use interval query to determine the corridor at target
        self.context.target_dist_bounds = self.forward_potential.init(source, target);

        if let Some((_, target_dist_upper)) = self.context.target_dist_bounds {
            // 2. initialize custom elimination tree
            let target = self.cch.node_order().rank(target);
            self.context.potentials.reset();
            self.context.backward_distances.reset();
            self.context.backward_distances[target as usize] = 0;

            let mut next = Some(target);
            while let Some(current_node) = next {
                next = self.cch.elimination_tree()[current_node as usize].value();

                // additional pruning: ignore node if the distance already exceeds the target dist bounds
                if self.context.backward_distances[current_node as usize] > target_dist_upper {
                    continue;
                }

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, current_node) {
                    let edge_id = edge as usize;
                    let next_node_orig = self.cch.node_order().node(next_node);

                    if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(next_node_orig) {
                        debug_assert!(target_dist_upper >= node_lower);

                        let start_idx = (((timestamp + node_lower) % MAX_BUCKETS) / self.interval_length) as usize;
                        let end_idx = (((timestamp + node_upper) % MAX_BUCKETS) / self.interval_length) as usize;

                        let mut idx = start_idx;
                        let mut edge_weight = *unsafe { self.backward_cch_weights.get_unchecked(idx * self.backward_cch_graph.num_arcs() + edge_id) };
                        while idx != end_idx {
                            idx = (idx + 1) % self.num_intervals as usize;
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
            let node = self.cch.node_order().rank(node);
            let elimination_tree = self.cch.elimination_tree();

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
                let current_node_orig = self.cch.node_order().node(current_node);
                // check if the current node is feasible, i.e. is able to reach the target within the valid corridor
                if let Some((node_lower, node_upper)) = self.forward_potential.potential_bounds(current_node_orig) {
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
                                idx = (idx + 1) % self.num_intervals as usize;
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
        let result = distance == INFINITY || self.context.target_dist_bounds.unwrap().1 >= distance;

        if !result {
            println!(
                "Result: {}, Bounds: {:?}",
                distance,
                self.context.target_dist_bounds.unwrap_or((INFINITY, INFINITY))
            );
        }

        result
    }
}
