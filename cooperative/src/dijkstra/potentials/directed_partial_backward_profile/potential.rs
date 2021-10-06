use std::borrow::Borrow;
use std::cmp::{max, min};

use crate::dijkstra::corridor_elimination_tree::customized::CustomizedUpperLower;
use crate::dijkstra::corridor_elimination_tree::server::CorridorEliminationTreeServer;
use crate::dijkstra::potentials::directed_partial_backward_profile::ops::{DirectedPartialBackwardProfileLabel, TDDirectedPartialBackwardProfilePotentialOps};
use crate::dijkstra::potentials::directed_partial_backward_profile::query::TDDirectedPartialBackwardProfileQuery;
use crate::dijkstra::potentials::{convert_timestamp_f64_to_u32, convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;
use rand::{thread_rng, Rng};
use rust_road_router::algo::ch_potentials::{CCHPotData, CCHPotential};
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, DijkstraRun, Label, State};
use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::floating_time_dependent::{period, FlWeight, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{Arc, BuildReversed, EdgeId, FirstOutGraph, Graph, LinkIterable, NodeId, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report::measure;
use crate::dijkstra::potentials::partial_backward_profile::ops::TDPartialBackwardProfilePotentialOps;

/// Basic implementation of a potential obtained by a backward profile search
/// this version is not to be used, but provides a good starting point for further optimizations
pub struct TDDirectedPartialBackwardProfilePotential<'a> {
    forward_cch: CorridorEliminationTreeServer,
    forward_lower_bound_potential: CCHPotential<'a, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>>, // for directed backward profile search
    backward_graph: ReversedGraphWithEdgeIds,
    backward_potential: CCHPotential<'a, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>>, // for directed backward profile search
    travel_time_profile: Vec<Vec<TTFPoint>>,
    dijkstra: DijkstraData<Vec<TTFPoint>>,
    is_visited: Vec<bool>,
    query_start: Timestamp,
    earliest_arrival: (Timestamp, Timestamp),
    /*is_pruned: Vec<bool>,
    is_visited: Vec<bool>,
    longitude: &'a Vec<f32>,
    latitude: &'a Vec<f32>,*/
}

impl<'a> TDDirectedPartialBackwardProfilePotential<'a> {
    pub fn new(graph: &CapacityGraph, cch: &'a CCH, cch_pot_data: &'a CCHPotData) -> Self {
        // copy graph structure for potentials
        let num_nodes = graph.num_nodes();

        // borrow departure and travel time
        let departure = graph.departure();
        let travel_time = graph.travel_time();

        // init forward cch
        let forward_cch = CorridorEliminationTreeServer::new(CustomizedUpperLower::new(cch, travel_time));
        let forward_lower_bound_potential = cch_pot_data.forward_potential();

        // init backward graph and potentials
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);
        let backward_potential = cch_pot_data.backward_potential();

        // create struct, customize and return
        let mut ret = Self {
            forward_cch,
            forward_lower_bound_potential,
            backward_graph,
            backward_potential,
            travel_time_profile: Vec::with_capacity(departure.len()), // customized below
            dijkstra: DijkstraData::new(num_nodes),
            is_visited: vec![false; num_nodes],
            query_start: Timestamp::ZERO,
            earliest_arrival: (Timestamp::ZERO, Timestamp::ZERO),
            /*is_pruned: vec![false; num_nodes],
            is_visited: vec![false; num_nodes],
            longitude,
            latitude,*/
        };
        ret.update_weights(departure, travel_time);
        ret
    }

    /// takes separate vectors of departure and time, merges them into TTFPoint vector
    pub fn update_weights(&mut self, departure: &Vec<Vec<u32>>, travel_time: &Vec<Vec<Weight>>) {
        self.travel_time_profile = departure
            .iter()
            .zip(travel_time.iter())
            .map(|(node_departure, node_travel_time)| {
                let edge_tt_profile = node_departure
                    .iter()
                    .zip(node_travel_time.iter())
                    .map(|(&ts, &val)| TTFPoint {
                        at: Timestamp::new(convert_timestamp_u32_to_f64(ts)),
                        val: FlWeight::new((val as f64) / 1000.0),
                    })
                    .collect::<Vec<TTFPoint>>();
                edge_tt_profile
            })
            .collect::<Vec<Vec<TTFPoint>>>();
    }
}

impl<'a> TDPotential for TDDirectedPartialBackwardProfilePotential<'a> {
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: u32) {
        println!("Query: {} -> {} (ts: {})", source, target, timestamp);

        // 1. retrieve lower/upper bound of earliest-arrival query
        let ((ea_dist_lower, ea_dist_upper), time) = measure(|| self.forward_cch.query(source, target).unwrap());
        println!("Earliest arrival query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
        println!("Distance Bounds: ({}, {})", ea_dist_lower, ea_dist_upper);

        let query = TDDirectedPartialBackwardProfileQuery {
            target,
            earliest_arrival_lower_bound: Timestamp(convert_timestamp_u32_to_f64(timestamp + ea_dist_lower)),
            earliest_arrival_upper_bound: Timestamp(convert_timestamp_u32_to_f64(timestamp + ea_dist_upper)),
        };
        //println!("Corridor at target: {:?}", &query.initial_state());

        self.query_start = Timestamp(convert_timestamp_u32_to_f64(timestamp));
        self.earliest_arrival = (query.earliest_arrival_lower_bound, query.earliest_arrival_upper_bound);

        // 2. SPECIAL CASE TREATMENT:
        // if the corridor is too small, then simply proceed with the lower bound potentials
        // the negligible deviations between lower and upper distance do not justify complex computations
        if ea_dist_lower == ea_dist_upper {
            self.forward_lower_bound_potential.init(source, target, timestamp);
            return;
        }

        // 3. init potentials for backward search
        self.backward_potential.init(target, source, timestamp);

        // 4. init ops with corridor data
        let mut ops = TDPartialBackwardProfilePotentialOps {
            query_start: self.query_start.clone(),
            corridor_max: query.earliest_arrival_upper_bound,
            profiles: &self.travel_time_profile
        };

        /*let mut ops = TDDirectedPartialBackwardProfilePotentialOps::new(
            Timestamp(convert_timestamp_u32_to_f64(timestamp)),
            query.earliest_arrival_upper_bound,
            &self.travel_time_profile,
            500,
        );*/

        // 5. init dijkstra run
        //self.is_pruned.iter_mut().for_each(|x| *x = false);
        self.is_visited.iter_mut().for_each(|x| *x = false);

        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();
        self.dijkstra.queue.push(State {
            key: FlWeight::ZERO,
            node: target,
        });
        self.dijkstra.distances[target as usize] = query.initial_state();

        // 6. run query
        let mut max_tent_dist = FlWeight::INFINITY; //bounds for max tentative distance to source node
        let mut counter = 0;

        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            let current_node_label = &mut self.dijkstra.distances[node as usize];
            self.is_visited[node as usize] = true;

            // approximate labels before they get too complex
            // ops.approximate_at_threshold(current_node_label);

            counter += 1;
            if counter % 10000 == 0 {
                println!("Extracted {} nodes", counter);
            }

            // whenever the source node is extracted (remember: backward search!),
            // make max distance as tight as possible
            if node == source {
                let node_max_dist = current_node_label.iter().max_by_key(|p| p.val).map(|&p| p.val).unwrap();
                max_tent_dist = min(max_tent_dist, node_max_dist);
                //dbg!(&current_node_label);
                //println!("Source node extracted! Step: {}, Max Dist: {}, Min dist: {}", counter, node_max_dist.0, current_node_label.min_dist.0);
            } else {
                let expected_dist = self
                    .backward_potential
                    .potential(node, 0)
                    .map(convert_timestamp_u32_to_f64)
                    .map(FlWeight)
                    .map(|p| p + current_node_label.iter().min_by_key(|x| x.val).map(|x| x.val).unwrap());

                if expected_dist.is_some() && max_tent_dist.fuzzy_lt(expected_dist.unwrap()) {
                    // pruning: ignore node if the max tentative distance to the source can't be beaten anymore,
                    // i.e. the current min dist already exceeds the source's max
                    //self.is_pruned[node as usize] = true;
                    continue;
                }
            }

            // relax outgoing edges
            for link in self.backward_graph.link_iter(node) {
                let linked = ops.link(&self.backward_graph, &self.dijkstra.distances[node as usize], &link);

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    let next_label = &self.dijkstra.distances[link.head() as usize];

                    let potential = self
                        .backward_potential
                        .potential(link.head(), 0)
                        .map(convert_timestamp_u32_to_f64)
                        .map(FlWeight);

                    if let Some(next_key) = potential.map(|p| p + next_label.iter().min_by_key(|x| x.val).map(|x| x.val).unwrap()) {
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

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        // special case treatment if corridor is too tight
        if self.earliest_arrival.0 == self.earliest_arrival.1 {
            println!("Using lowerbound potential");
            return self.forward_lower_bound_potential.potential(node, timestamp);
        }

        // else, proceed with normal interpolation
        let node_label = &self.dijkstra.distances[node as usize];
        let timestamp = Timestamp(convert_timestamp_u32_to_f64(timestamp));

        let first_label_entry = node_label.first().unwrap();
        let last_label_entry = node_label.last().unwrap();

        // check if timestamp is inside the profile's boundaries
        if node_label.first().unwrap().at.fuzzy_leq(timestamp) && timestamp.fuzzy_leq(node_label.last().unwrap().at) {
            // interpolate between the respective profile breakpoints
            let pot = PartialPiecewiseLinearFunction::new(&node_label).eval(timestamp);
            Some(convert_timestamp_f64_to_u32(pot.0))
        } else {
            None
        }

        /*else if !self.is_visited[node as usize]
            || (last_label_entry.at.fuzzy_lt(timestamp) && self.earliest_arrival.1.fuzzy_eq(last_label_entry.at + last_label_entry.val))
        {
            // so far, there are two valid cases in which the potential doesn't match with the given timestamp
            // 1) node hasn't been visited at all due to pruning
            // 2) earliest possible arrival time at target node is outside of the query's corridor
            None
        } else {
            // otherwise, there is no potential for this node
            // this could happen if the profile corridors are set too tightly
            dbg!(
                &self.query_start,
                &self.earliest_arrival,
                &timestamp,
                &node_label.ttf.last(),
                &node_label.min_dist,
                &self.backward_potential.potential(node, 0)
            );

            panic!(
                "Failed to find the potential for {} at ts {}: Partial profile spans range [{}, {}] (lower: {}, upper: {}, visited: {})",
                node,
                timestamp.0,
                node_label.ttf.first().unwrap().at.0,
                node_label.ttf.last().unwrap().at.0,
                self.dijkstra.distances[node as usize].ttf_start.0,
                self.dijkstra.distances[node as usize].ttf_end.0,
                self.is_visited[node as usize],
            )
        }*/
    }
}
