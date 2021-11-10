use crate::dijkstra::elimination_tree::approximated_periodic_ttf::customized::CustomizedApproximatedPeriodicTTF;
use crate::dijkstra::elimination_tree::approximated_periodic_ttf::CorridorLowerboundEliminationTreeWalk;
use crate::dijkstra::potentials::cch::custom_cch_pot::CCHLowerUpperPotential;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::cmp::min;

const LOWERBOUND_POT_THRESHOLD: Timestamp = 600_000; // = 10 minutes

pub struct CorridorLowerboundPotential<'a> {
    customized: &'a CustomizedApproximatedPeriodicTTF<'a>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<Weight>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Vec<Weight>>,
    backward_distances: TimestampedVector<Weight>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Vec<Weight>>,
    forward_potential: CCHLowerUpperPotential<'a, CCH>,
    num_pot_computations: usize,
    target_dist_bounds: (Weight, Weight),
    query_start: u32,
    use_simple_pot: bool,
}

impl<'a> CorridorLowerboundPotential<'a> {
    pub fn new(customized: &'a CustomizedApproximatedPeriodicTTF<'a>) -> Self {
        let (forward_cch_graph, forward_cch_weights, forward_cch_bounds) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights, backward_cch_bounds) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        let forward_potential = CCHLowerUpperPotential::new_forward(customized.cch, forward_cch_bounds.clone(), backward_cch_bounds.clone());

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_potential,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            backward_distances: TimestampedVector::new(n),
            num_pot_computations: 0,
            target_dist_bounds: (INFINITY, INFINITY),
            query_start: 0,
            use_simple_pot: false,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }
}

impl<'a> TDPotential for CorridorLowerboundPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        // 1. use interval query to determine the corridor
        self.forward_potential.init(source, target, timestamp);
        let (target_dist_lower, target_dist_upper) = self.forward_potential.potential_bounds(source).unwrap();

        //println!("Interval query bounds: {} - {}", target_dist_lower, target_dist_upper);
        self.query_start = timestamp;
        self.target_dist_bounds = (target_dist_lower, target_dist_upper);

        // use the simple forward potential as fallback if the corridor is too tight
        // the extra work invested will not improve the potential quality significantly
        if target_dist_upper - target_dist_lower <= LOWERBOUND_POT_THRESHOLD {
            self.use_simple_pot = true;
            return;
        } else {
            self.use_simple_pot = false;
        }

        // 2. initialize custom elimination tree
        let interval_length = MAX_BUCKETS / self.customized.num_intervals;
        let target = self.customized.cch.node_order().rank(target);
        self.potentials.reset();

        let mut bw_walk = CorridorLowerboundEliminationTreeWalk::init(
            &self.backward_cch_graph,
            self.customized.cch.elimination_tree(),
            &mut self.backward_distances,
            target,
        );

        while let Some((_, outgoing_edges)) = bw_walk.peek() {
            // get the correct weights for the current outgoing edge
            let mut outgoing_links = Vec::with_capacity(outgoing_edges.len());
            for (next_node, edge_id) in outgoing_edges {
                let edge_id = edge_id.0 as usize;
                let next_node_orig_id = self.customized.cch.node_order.node(next_node.0);

                let next_node_pot = self.forward_potential.potential_bounds(next_node_orig_id).and_then(|(node_lower, node_upper)| {
                    // ignore an edge if its head violates the target corridor
                    if target_dist_upper < node_lower {
                        None
                    } else {
                        let start = if target_dist_lower > node_upper {
                            timestamp + target_dist_lower - node_upper
                        } else {
                            timestamp
                        };

                        Some((
                            ((start % MAX_BUCKETS) / interval_length) as usize,
                            (((timestamp + target_dist_upper - node_lower) % MAX_BUCKETS) / interval_length) as usize,
                        ))
                    }
                });

                if let Some((start_interval, end_interval)) = next_node_pot {
                    // deal with edge case: intervals might cross midnight
                    let interval_min = if start_interval <= end_interval {
                        // easy case: work on one consecutive slice
                        self.backward_cch_weights[edge_id][start_interval..=end_interval].iter().min().cloned().unwrap()
                    } else {
                        // complex case, take minimum from `start_interval` to midnight and midnight to `end_interval`
                        min(
                            self.backward_cch_weights[edge_id][start_interval..].iter().min().cloned().unwrap(),
                            self.backward_cch_weights[edge_id][..=end_interval].iter().min().cloned().unwrap(),
                        )
                    };

                    outgoing_links.push((next_node, interval_min));
                }
            }

            bw_walk.next(&outgoing_links);
        }
        self.num_pot_computations = 0;
    }

    fn potential(&mut self, node: u32, _timestamp: u32) -> Option<u32> {
        // additional case distinction: if the corridor is too tight,
        // simply fall back to the faster lowerbound potential
        if self.use_simple_pot {
            self.forward_potential.potential(node, _timestamp).filter(|&pot| pot < INFINITY)
        } else {
            let interval_length = MAX_BUCKETS / self.customized.num_intervals;
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
                // convert back to original id in order to use it in the other CCH
                let current_node_pot_intervals = self
                    .forward_potential
                    .potential_bounds(self.customized.cch.node_order.node(current_node))
                    .and_then(|(node_lower, node_upper)| {
                        if self.target_dist_bounds.1 < node_lower {
                            None
                        } else {
                            let start = if self.target_dist_bounds.0 > node_upper {
                                self.query_start + self.target_dist_bounds.0 - node_upper
                            } else {
                                self.query_start
                            };

                            Some((
                                ((start % MAX_BUCKETS) / interval_length) as usize,
                                (((self.query_start + self.target_dist_bounds.1 - node_lower) % MAX_BUCKETS) / interval_length) as usize,
                            ))
                        }
                    });

                // only proceed if the node is feasible, otherwise return `None`
                if let Some((start_interval, end_interval)) = current_node_pot_intervals {
                    for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                        // even in the forward direction, we're still performing backward linking,
                        // current edges are all starting at `current_node`
                        // -> take the same edge interval of all outgoing edges as given by the corridor

                        // cover edge case: intervals span over midnight
                        let edge_interval_min = if start_interval <= end_interval {
                            *self.forward_cch_weights[edge as usize][start_interval..=end_interval].iter().min().unwrap()
                        } else {
                            min(
                                *self.forward_cch_weights[edge as usize][start_interval..].iter().min().unwrap(),
                                *self.forward_cch_weights[edge as usize][..=end_interval].iter().min().unwrap(),
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
        }
    }
}
