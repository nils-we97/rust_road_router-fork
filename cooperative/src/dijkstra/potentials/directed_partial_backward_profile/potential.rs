use std::cmp::max;

use crate::dijkstra::potentials::directed_partial_backward_profile::ops::{DirectedPartialBackwardProfileLabel, TDDirectedPartialBackwardProfilePotentialOps};
use crate::dijkstra::potentials::partial_backward_profile::query::TDPartialBackwardProfileQuery;
use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;
use rust_road_router::algo::ch_potentials::{CCHPotData, CCHPotential};
use rust_road_router::algo::dijkstra::{DefaultOps, DijkstraData, DijkstraOps, Label, Server, State};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{Arc, BuildReversed, EdgeId, FirstOutGraph, Graph, LinkIterable, NodeId, OwnedGraph, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::index_heap::Indexing;

/// Basic implementation of a potential obtained by a backward profile search
/// this version is not to be used, but provides a good starting point for further optimizations
pub struct TDDirectedPartialBackwardProfilePotential<'a> {
    forward_server: Server<
        OwnedGraph,
        DefaultOps,
        CCHPotential<'a, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>, FirstOutGraph<&'a [EdgeId], &'a [NodeId], &'a [Weight]>>,
        OwnedGraph,
    >,
    backward_graph: ReversedGraphWithEdgeIds,
    lowerbound_potential: &'a CCHPotData, // for directed backward profile search
    travel_time_profile: Vec<Vec<TTFPoint>>,
    dijkstra: DijkstraData<DirectedPartialBackwardProfileLabel>,
}

impl<'a> TDDirectedPartialBackwardProfilePotential<'a> {
    pub fn new(graph: &CapacityGraph, cch_pot_data: &'a CCHPotData) -> Self {
        // copy graph structure for potentials
        let num_nodes = graph.num_nodes();
        let first_out = graph.first_out().to_vec();
        let head = graph.head().to_vec();
        let weight = graph.freeflow_time().to_vec();

        // borrow departure and travel time
        let departure = graph.departure();
        let travel_time = graph.travel_time();

        // init forward and backward graph
        let forward_graph = OwnedGraph::new(first_out, head, weight);
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);
        let forward_server = Server::with_potential(forward_graph, cch_pot_data.forward_potential());

        // create struct, customize and return
        let mut ret = Self {
            forward_server,
            backward_graph,
            lowerbound_potential: cch_pot_data,
            travel_time_profile: Vec::with_capacity(departure.len()), // customized below
            dijkstra: DijkstraData::new(num_nodes),
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
        let earliest_arrival_distance = self.forward_server.query(Query::new(source, target, 0)).distance().unwrap();

        // 2. initialize backwards profile dijkstra: custom implementation with additional pruning
        // initial time corridor: 2 * lowerbound travel time, but at least 30 minutes
        let query = TDPartialBackwardProfileQuery {
            target,
            earliest_arrival: Timestamp(convert_timestamp_u32_to_f64(timestamp + earliest_arrival_distance)),
            initial_timeframe: Timestamp(convert_timestamp_u32_to_f64(max(2 * earliest_arrival_distance, 1800))),
        };

        // 3. init ops with corridor data
        let mut ops = TDDirectedPartialBackwardProfilePotentialOps {
            query_start: Timestamp(convert_timestamp_u32_to_f64(timestamp)),
            corridor_max: Timestamp(query.earliest_arrival.0 + query.initial_timeframe.0),
            profiles: &self.travel_time_profile,
        };

        // 4. init dijkstra run
        let mut max_tent_dist = FlWeight::INFINITY; //bounds for max tentative distance to source node
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
        let mut backward_potential = self.lowerbound_potential.backward_potential();
        backward_potential.init(target, source, timestamp);

        // 6. run query
        let mut counter = 0;
        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            counter += 1;
            if counter % 10000 == 0 {
                println!("Finished {} nodes", counter);
            }

            let current_node_label = &self.dijkstra.distances[node as usize];

            // whenever the source node is extracted (remember: backward search!),
            // make max distance as tight as possible
            if node == source {
                let node_max_dist = current_node_label.ttf.iter().map(|&p| p.val).max().unwrap();

                if node_max_dist.fuzzy_lt(max_tent_dist) {
                    max_tent_dist = node_max_dist;
                }
            } else if max_tent_dist.fuzzy_lt(FlWeight::INFINITY) {
                // pruning: ignore node if the max tentative distance to the source can't be beaten anymore,
                // i.e. the current min dist already exceeds the source's max
                let current_node_min_dist = current_node_label.ttf.iter().map(|p| p.val).min().unwrap();

                if max_tent_dist.fuzzy_lt(current_node_min_dist) {
                    //dbg!("Pruned node!", current_node_min_dist, &max_tent_dist);
                    continue;
                }
            }

            // relax outgoing edges
            for link in self.backward_graph.link_iter(node) {
                let linked = ops.link(&self.backward_graph, &self.dijkstra.distances[node as usize], &link);

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    let next_label = &self.dijkstra.distances[link.head() as usize];

                    let potential = backward_potential.potential(link.head(), 0).map(convert_timestamp_u32_to_f64).map(FlWeight);
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
        //let (_, time) = measure(|| while let Some(_node) = run.next() {});
        //println!("Potential init took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        let node_profile = &self.dijkstra.distances[node as usize].ttf;
        let timestamp = Timestamp(convert_timestamp_u32_to_f64(timestamp));

        // check if timestamp is inside the profile's boundaries
        if node_profile.first().unwrap().at.fuzzy_leq(timestamp) && timestamp.fuzzy_leq(node_profile.last().unwrap().at) {
            // interpolate between the respective profile breakpoints
            let pot = PartialPiecewiseLinearFunction::new(node_profile).eval(timestamp);
            Some((1000.0 * pot.0) as u32)
        } else {
            // otherwise, there is no potential for this node
            // this could happen if the profile corridors are set too tightly
            panic!(
                "Failed to find the potential for {} at ts {}: Partial profile spans range [{}, {}]",
                node,
                timestamp.0,
                node_profile.first().unwrap().at.0,
                node_profile.last().unwrap().at.0
            )
        }
    }
}
