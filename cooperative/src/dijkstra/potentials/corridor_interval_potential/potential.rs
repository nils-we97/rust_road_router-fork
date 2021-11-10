use crate::dijkstra::elimination_tree::corridor_intervals::customized::CustomizedUpperLower;
use crate::dijkstra::potentials::cch::custom_cch_pot::CCHLowerUpperPotential;
use crate::dijkstra::potentials::corridor_interval_potential::ops::{ApproximatedIntervalLabel, IntervalLabelEntry, TDCorridorIntervalPotentialOps};
use crate::dijkstra::potentials::TDPotential;
use crate::graph::capacity_graph::CapacityGraph;
use rust_road_router::algo::ch_potentials::{CCHPotData, CCHPotential};
use rust_road_router::algo::customizable_contraction_hierarchy::DirectedCCH;
use rust_road_router::algo::dijkstra::{DijkstraData, Label, State};
use rust_road_router::datastr::graph::{Arc, BuildReversed, EdgeId, FirstOutGraph, Graph, LinkIterable, NodeId, ReversedGraphWithEdgeIds, Weight, INFINITY};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::cmp::min;

/// Approximated Interval Potentials, limited by a corridor for each node
/// Results might not be as precise as partial profiles, however they can be calculated significantly faster
/// as link and merge are basically O(1) operations
pub struct TDCorridorIntervalPotential<'a> {
    forward_potential: CCHLowerUpperPotential<'a, DirectedCCH>,
    backward_graph: ReversedGraphWithEdgeIds,
    backward_lower_bound_potential:
        CCHPotential<'a, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>>,

    dijkstra: DijkstraData<ApproximatedIntervalLabel>,
    ops: TDCorridorIntervalPotentialOps,
    lazy_corridors: TimestampedVector<Option<(Weight, Weight)>>,
    use_td_mode: bool,
}

impl<'a> TDCorridorIntervalPotential<'a> {
    pub fn new(graph: &CapacityGraph, customized_upper_lower: &'a CustomizedUpperLower, cch_pot_data: &'a CCHPotData, num_intervals: u32) -> Self {
        // copy graph structure for potentials
        let num_nodes = graph.num_nodes();

        // borrow departure and travel time, customize the ops
        let ops = TDCorridorIntervalPotentialOps::new(num_intervals, graph.departure(), graph.travel_time());

        // init forward potential for queries with a tight corridor and corridor bounds
        let forward_potential = CCHLowerUpperPotential::new_forward(
            &customized_upper_lower.cch,
            customized_upper_lower.upward.clone(),
            customized_upper_lower.downward.clone(),
        );

        // init backward graph and potentials
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);
        let backward_lower_bound_potential = cch_pot_data.backward_potential();

        Self {
            forward_potential,
            backward_graph,
            backward_lower_bound_potential,
            dijkstra: DijkstraData::new(num_nodes),
            ops,
            use_td_mode: true,
            lazy_corridors: TimestampedVector::new(num_nodes),
        }
    }

    /// customization: update edge interval weights
    pub fn update_weights(&mut self, departures: &Vec<Vec<u32>>, travel_times: &Vec<Vec<Weight>>) {
        self.ops = TDCorridorIntervalPotentialOps::new(self.ops.num_intervals, departures, travel_times);
    }
}

impl<'a> TDPotential for TDCorridorIntervalPotential<'a> {
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: u32) {
        //println!("Query: {} -> {} (ts: {})", source, target, timestamp);

        // 1. initialize forward potential and retrieve corridor from interval query
        self.forward_potential.init(source, target, timestamp);
        let (ea_dist_lower, ea_dist_upper) = self.forward_potential.potential_bounds(source).unwrap();

        // 2. special case treatment: if both lower and upper bound arrival time are inside the same interval,
        // then simply proceed with the lowerbound forward potential
        let target_begin_interval = self.ops.interval_length * ((timestamp + ea_dist_lower) / self.ops.interval_length);
        let target_end_interval = self.ops.interval_length * ((timestamp + ea_dist_upper) / self.ops.interval_length);

        if target_begin_interval == target_end_interval {
            self.use_td_mode = false;
            return;
        }
        //println!("use td mode");
        self.use_td_mode = true;

        // 3. init potentials for backward search
        self.backward_lower_bound_potential.init(target, source, timestamp);

        // 4. reset dijkstra context
        self.lazy_corridors.reset();
        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();
        self.dijkstra.queue.push(State { key: 0, node: target });
        self.lazy_corridors[target as usize] = Some((timestamp + ea_dist_lower, timestamp + ea_dist_upper));

        // 5. set target label
        let num_entries = ((target_end_interval - target_begin_interval) / self.ops.interval_length) + 1;

        self.dijkstra.distances[target as usize] =
            ApproximatedIntervalLabel::new(Some(target_begin_interval), vec![IntervalLabelEntry::new(0, None); num_entries as usize]);

        // 6. run query
        let mut max_tent_dist = INFINITY; //bounds for max tentative distance to source node
        let mut counter = 0;

        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            let current_node_label = &mut self.dijkstra.distances[node as usize];

            counter += 1;
            if counter % 100000 == 0 {
                println!("Extracted {} nodes", counter);
            }

            // whenever the source node is extracted (remember: backward search!),
            // make max distance as tight as possible
            if node == source {
                max_tent_dist = min(max_tent_dist, current_node_label.max_dist());
            } else {
                // pruning: ignore node if the max tentative distance to the source can't be beaten anymore,
                // even by the smallest distance
                let expected_dist = self.backward_lower_bound_potential.potential(node, 0).map(|p| p + current_node_label.key());

                if expected_dist.is_some() && max_tent_dist < expected_dist.unwrap() {
                    continue;
                }
            }

            // relax outgoing edges
            for link in self.backward_graph.link_iter(node) {
                // retrieve the corridor of the next node, and lazily store this value
                let node_corridor = if self.lazy_corridors[link.head() as usize].is_some() {
                    self.lazy_corridors[link.head() as usize]
                } else {
                    let pot = self.forward_potential.potential_bounds(link.head()).and_then(|(lower, upper)| {
                        // lower and upper are bounds towards the target
                        // in order to use them for the backward search, we have to subtract them
                        // from the earliest arrival query

                        // earliest possible departure: lower bound to target - upper dist
                        // latest possible departure: upper bound to target - lower dist

                        // also do additional pruning if the current edge yields into the completely wrong direction
                        if ea_dist_upper < lower {
                            None
                        } else {
                            let lower_corridor = if ea_dist_lower > upper {
                                timestamp + ea_dist_lower - upper
                            } else {
                                timestamp
                            };

                            Some((lower_corridor, timestamp + ea_dist_upper - lower))
                        }
                    });

                    self.lazy_corridors[link.head() as usize] = pot;
                    pot
                };

                // only proceed if the vertex has valid upper and lower bounds
                if let Some(corridor) = node_corridor {
                    let linked = self.ops.link_in_bounds(&self.dijkstra.distances[node as usize], link.1, corridor);
                    /*if link.head() == 36012 {
                        dbg!(
                            &self.ops.edge_intervals[link.1 .0 .0 as usize],
                            &self.dijkstra.distances[node as usize],
                            &corridor,
                            &self.ops.interval_length
                        );
                        println!("Linked result: {:?}, current label: {:?}", linked, &self.dijkstra.distances[36012]);
                    }*/

                    if self.ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                        let next_label = &self.dijkstra.distances[link.head() as usize];
                        let potential = self.backward_lower_bound_potential.potential(link.head(), 0);

                        if let Some(next_key) = potential.map(|p| p + next_label.key()) {
                            let next = State {
                                node: link.head(),
                                key: next_key,
                            };

                            if self.dijkstra.queue.contains_index(next.as_index()) {
                                self.dijkstra.queue.decrease_key(next);
                            } else {
                                self.dijkstra.queue.push(next);
                            }
                        }
                    }
                }
            }
        }
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        // special case treatment if corridor is too tight
        if self.use_td_mode {
            let node_label = &self.dijkstra.distances[node as usize];

            // check if the timestamp is inside the profile's supposed boundaries
            if let Some((corridor_lower, corridor_upper)) = self.lazy_corridors[node as usize] {
                debug_assert!(
                    timestamp >= corridor_lower,
                    "timestamp must definitely be larger than the lower corridor: node {}, ts {} corridor {} - {}",
                    node,
                    timestamp,
                    corridor_lower,
                    corridor_upper
                );

                if timestamp <= corridor_upper && node_label.first_interval_ts.is_some() {
                    // timestamp is inside boundaries, therefore a valid result is expected
                    debug_assert!(
                        timestamp >= node_label.first_interval_ts.unwrap(),
                        "Label entry should exist at the given timestamp! ts: {}, corridor: {} - {}",
                        timestamp,
                        corridor_lower,
                        corridor_upper
                    );

                    // round timestamp to enable valid indexing
                    let idx = ((timestamp / self.ops.interval_length) - (node_label.first_interval_ts.unwrap() / self.ops.interval_length)) as usize;
                    node_label
                        .interval_minima
                        .get(idx)
                        .map(|val| val.overflow_min.map(|(overflow_val, _)| overflow_val).unwrap_or(val.no_overflow_min))

                    /*debug_assert!(
                        result.is_some(),
                        "Expected valid entry at node {} and ts {}, corridor bounds: {} - {}, {:?}",
                        node,
                        timestamp,
                        corridor_lower,
                        corridor_upper,
                        self.forward_potential.potential(node, timestamp)
                    );*/
                } else {
                    /*println!(
                        "Timestamp > upper corridor - {:?}; {}, {} - {}",
                        self.forward_potential.potential(node, timestamp),
                        timestamp,
                        corridor_lower,
                        corridor_upper
                    );*/
                    None
                }
            } else {
                // node is not relevant
                //println!("Node not relevant - {:?}, {}", self.forward_potential.potential(node, timestamp), timestamp);
                None
            }
        } else {
            self.forward_potential.potential(node, timestamp)
        }
    }
}
