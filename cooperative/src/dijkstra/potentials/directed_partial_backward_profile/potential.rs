use std::cmp::{max, min};

use crate::dijkstra::potentials::directed_partial_backward_profile::ops::{DirectedPartialBackwardProfileLabel, TDDirectedPartialBackwardProfilePotentialOps};
use crate::dijkstra::potentials::partial_backward_profile::query::TDPartialBackwardProfileQuery;
use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;
use rand::{thread_rng, Rng};
use rust_road_router::algo::ch_potentials::{CCHPotData, CCHPotential};
use rust_road_router::algo::customizable_contraction_hierarchy::query::Server;
use rust_road_router::algo::customizable_contraction_hierarchy::{customize, customize_perfect, DirectedCCH, CCH};
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{Arc, BuildReversed, EdgeId, FirstOutGraph, Graph, LinkIterable, NodeId, OwnedGraph, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report::measure;

/// Basic implementation of a potential obtained by a backward profile search
/// this version is not to be used, but provides a good starting point for further optimizations
pub struct TDDirectedPartialBackwardProfilePotential<'a> {
    forward_cch: Server<DirectedCCH, DirectedCCH>,
    backward_graph: ReversedGraphWithEdgeIds,
    backward_potential: CCHPotential<'a, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>>, // for directed backward profile search
    travel_time_profile: Vec<Vec<TTFPoint>>,
    dijkstra: DijkstraData<DirectedPartialBackwardProfileLabel>,
    is_pruned: Vec<bool>,
    is_visited: Vec<bool>,
    longitude: &'a Vec<f32>,
    latitude: &'a Vec<f32>,
}

impl<'a> TDDirectedPartialBackwardProfilePotential<'a> {
    pub fn new(graph: &CapacityGraph, cch: &'a CCH, cch_pot_data: &'a CCHPotData, longitude: &'a Vec<f32>, latitude: &'a Vec<f32>) -> Self {
        // copy graph structure for potentials
        let num_nodes = graph.num_nodes();
        let first_out = graph.first_out().to_vec();
        let head = graph.head().to_vec();
        let weight = graph.freeflow_time().to_vec();

        // borrow departure and travel time
        let departure = graph.departure();
        let travel_time = graph.travel_time();

        // init forward cch
        let forward_graph = OwnedGraph::new(first_out, head, weight);
        let forward_cch = Server::new(customize_perfect(customize(cch, &forward_graph)));

        // init backward graph and potentials
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);
        let backward_potential = cch_pot_data.backward_potential();

        // create struct, customize and return
        let mut ret = Self {
            forward_cch,
            backward_graph,
            backward_potential,
            travel_time_profile: Vec::with_capacity(departure.len()), // customized below
            dijkstra: DijkstraData::new(num_nodes),
            is_pruned: vec![false; num_nodes],
            is_visited: vec![false; num_nodes],
            longitude,
            latitude,
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
        // 1. earliest arrival query on lowerbound graph
        let (earliest_arrival_distance, time) = measure(|| self.forward_cch.query(Query::new(source, target, 0)).distance().unwrap());
        println!("Earliest arrival query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        // 2. initialize backwards profile dijkstra: custom implementation with additional pruning
        // initial time corridor: 2 * lowerbound travel time, but at least 30 minutes
        let query = TDPartialBackwardProfileQuery {
            target,
            earliest_arrival: Timestamp(convert_timestamp_u32_to_f64(timestamp + earliest_arrival_distance)),
            initial_timeframe: Timestamp(convert_timestamp_u32_to_f64(max(2 * earliest_arrival_distance, 1800))),
        };

        println!("Query: {} -> {} (ts: {}), corridor: {:?}", source, target, timestamp, query.initial_state());

        // 3. init ops with corridor data
        let mut ops = TDDirectedPartialBackwardProfilePotentialOps {
            query_start: Timestamp(convert_timestamp_u32_to_f64(timestamp)),
            corridor_max: Timestamp(query.earliest_arrival.0 + query.initial_timeframe.0),
            profiles: &self.travel_time_profile,
        };

        // 4. init dijkstra run
        self.is_pruned.iter_mut().for_each(|x| *x = false);
        self.is_visited.iter_mut().for_each(|x| *x = false);

        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();
        self.dijkstra.queue.push(State {
            key: query.initial_state().key(),
            node: target,
        });
        self.dijkstra.distances[target as usize] = DirectedPartialBackwardProfileLabel {
            ttf: query.initial_state(),
            min_dist: FlWeight::ZERO,
        };

        // 5. init potentials for backward search
        self.backward_potential.init(target, source, timestamp);

        // 6. run query
        let mut max_tent_dist = FlWeight::INFINITY; //bounds for max tentative distance to source node
        let mut counter = 0;
        let mut num_adjusted_labels = 0;

        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            let current_node_label = &self.dijkstra.distances[node as usize];
            self.is_visited[node as usize] = true;

            counter += 1;
            if counter % 10000 == 0 {
                //println!("Extracted {} nodes - current min dist: {}", counter, current_node_label.min_dist.0);
            }

            // whenever the source node is extracted (remember: backward search!),
            // make max distance as tight as possible
            if node == source {
                let node_max_dist = current_node_label.ttf.iter().map(|&p| p.val).max().unwrap();
                max_tent_dist = min(max_tent_dist, node_max_dist);
                //println!("Source node extracted! Step: {}, Max Dist: {}", counter, max_tent_dist.0);
            } else {
                let expected_dist = self
                    .backward_potential
                    .potential(node, 0)
                    .map(convert_timestamp_u32_to_f64)
                    .map(FlWeight)
                    .map(|p| p + current_node_label.min_dist);

                if expected_dist.is_some() && max_tent_dist.fuzzy_lt(expected_dist.unwrap()) {
                    // pruning: ignore node if the max tentative distance to the source can't be beaten anymore,
                    // i.e. the current min dist already exceeds the source's max
                    //let current_node_min_dist = current_node_label.ttf.iter().map(|p| p.val).min().unwrap();
                    self.is_pruned[node as usize] = true;
                    continue;
                }
            }

            // relax outgoing edges
            for link in self.backward_graph.link_iter(node) {
                let linked = ops.link(&self.backward_graph, &self.dijkstra.distances[node as usize], &link);

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    num_adjusted_labels += 1;
                    let next_label = &self.dijkstra.distances[link.head() as usize];

                    let potential = self
                        .backward_potential
                        .potential(link.head(), 0)
                        .map(convert_timestamp_u32_to_f64)
                        .map(FlWeight);
                    if let Some(next_key) = potential.map(|p| p + next_label.min_dist) {
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

        let num_pruned_nodes = self.is_pruned.iter().filter(|&&x| x).count();
        let num_visited_nodes = self.is_visited.iter().filter(|&&x| x).count();
        println!(
            "Visited nodes: {} / {}, pruned nodes: {}, adjusted labels: {}",
            num_visited_nodes,
            self.backward_graph.num_nodes(),
            num_pruned_nodes,
            num_adjusted_labels
        );

        // visualization
        let mut rng = thread_rng();
        println!("let source = [{}, {}];", self.latitude[source as usize], self.longitude[source as usize]);
        println!("let target = [{}, {}];", self.latitude[target as usize], self.longitude[target as usize]);
        println!("[");
        self.is_visited
            .iter()
            .enumerate()
            .filter(|&(_, &v)| v && rng.gen_range(0..100) == 0)
            .for_each(|(idx, _)| {
                println!("[{}, {}],", self.latitude[idx], self.longitude[idx]);
            });
        println!("]");
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        let node_profile = &self.dijkstra.distances[node as usize].ttf;
        let timestamp = Timestamp(convert_timestamp_u32_to_f64(timestamp));

        // check if timestamp is inside the profile's boundaries
        if node_profile.first().unwrap().at.fuzzy_leq(timestamp) && timestamp.fuzzy_leq(node_profile.last().unwrap().at) {
            // interpolate between the respective profile breakpoints
            let pot = PartialPiecewiseLinearFunction::new(node_profile).eval(timestamp);
            Some((1000.0 * pot.0) as u32)
        } else if !self.is_visited[node as usize] {
            // due to pruning and merging/linking in a tight corridor,
            // some nodes might not be visited at all. For now, it's sufficient to ignore them during the query
            None
        } else {
            // otherwise, there is no potential for this node
            // this could happen if the profile corridors are set too tightly
            panic!(
                "Failed to find the potential for {} at ts {}: Partial profile spans range [{}, {}] (pruned: {}, visited: {})",
                node,
                timestamp.0,
                node_profile.first().unwrap().at.0,
                node_profile.last().unwrap().at.0,
                self.is_pruned[node as usize],
                self.is_visited[node as usize]
            )
        }
    }
}
