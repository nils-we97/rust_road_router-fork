use std::borrow::Borrow;
use std::cmp::max;

use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraRun, Server};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{BuildReversed, EdgeId, FirstOutGraph, Graph, NodeId, ReversedGraphWithEdgeIds, Weight};

use crate::dijkstra::potentials::partial_backward_profile::ops::TDPartialBackwardProfilePotentialOps;
use crate::dijkstra::potentials::partial_backward_profile::query::TDPartialBackwardProfileQuery;
use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;

/// Partial backward profile potential: restrict the backward search to a relevant time corridor
/// .. to be accelerated

pub struct TDPartialBackwardProfilePotential {
    forward_server: Server<FirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>>, // TODO use CCH for further speedup
    backward_graph: ReversedGraphWithEdgeIds,
    travel_time_profile: Vec<Vec<TTFPoint>>,
    dijkstra: DijkstraData<Vec<TTFPoint>>,
}

impl TDPartialBackwardProfilePotential {
    pub fn new(graph: &CapacityGraph) -> Self {
        let num_nodes = graph.num_nodes();

        // init forward graph
        let first_out = graph.first_out().to_vec();
        let head = graph.head().to_vec();
        let weight = graph.freeflow_time().to_vec();
        let lowerbound_forward_graph = FirstOutGraph::new(first_out, head, weight);
        let forward_server = Server::new(lowerbound_forward_graph);

        // init backward graph
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);

        let departure = graph.departure();
        let travel_time = graph.travel_time();

        let mut ret = Self {
            forward_server,
            backward_graph,
            travel_time_profile: Vec::new(), // placeholder, filled below
            dijkstra: DijkstraData::new(num_nodes),
        };
        ret.update_weights(&departure, &travel_time);
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

impl TDPotential for TDPartialBackwardProfilePotential {
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: u32) {
        // 1. earliest arrival query on lowerbound graph
        let earliest_arrival_distance = self.forward_server.query(Query::new(source, target, 0)).distance().unwrap();

        // initialize backwards profile dijkstra
        // initial time corridor: 2 * lowerbound travel time, but at least 30 minutes
        let query = TDPartialBackwardProfileQuery {
            target,
            earliest_arrival: Timestamp(convert_timestamp_u32_to_f64(timestamp + earliest_arrival_distance)),
            initial_timeframe: Timestamp(convert_timestamp_u32_to_f64(max(2 * earliest_arrival_distance, 1800))),
        };

        let mut ops = TDPartialBackwardProfilePotentialOps {
            query_start: Timestamp(convert_timestamp_u32_to_f64(timestamp)),
            corridor_max: Timestamp(query.earliest_arrival.0 + query.initial_timeframe.0),
            profiles: &self.travel_time_profile,
        };

        dbg!(&ops.query_start, &ops.corridor_max, &query.earliest_arrival, source, target);
        let mut run = DijkstraRun::query(self.backward_graph.borrow(), &mut self.dijkstra, &mut ops, query);

        // run through the whole graph
        let mut counter = 0;
        while let Some(_node) = run.next() {
            counter += 1;
            if counter % 10000 == 0 {
                println!("Finished {} nodes", counter);
            }
        }
        //let (_, time) = measure(|| while let Some(_node) = run.next() {});
        //println!("Potential init took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        let node_profile = &self.dijkstra.distances[node as usize];
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

/* ------------------------------------------------------------------------------------------- */
