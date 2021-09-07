use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::MAX_BUCKETS;
use crate::util::profile_search::find_profile_index;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, DijkstraRun, Label, Server};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::floating_time_dependent::{
    ATTFContainer, FlWeight, PartialATTF, PartialPiecewiseLinearFunction, PeriodicPiecewiseLinearFunction, TTFPoint, Timestamp,
};
use rust_road_router::datastr::graph::{BuildReversed, EdgeId, FirstOutGraph, Graph, NodeId, NodeIdT, Reversed, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::report::measure;
use std::borrow::Borrow;
use std::cmp::{max, min};

/// Basic implementation of a potential obtained by a backward profile search
/// this version is not to be used, but provides a good starting point for further optimizations

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
                node_departure
                    .iter()
                    .zip(node_travel_time.iter())
                    .map(|(&ts, &val)| TTFPoint {
                        at: Timestamp::new(convert_timestamp_u32_to_f64(ts)),
                        val: FlWeight::new((val as f64) / 1000.0),
                    })
                    .collect::<Vec<TTFPoint>>()
            })
            .collect::<Vec<Vec<TTFPoint>>>();
    }
}

impl TDPotential for TDPartialBackwardProfilePotential {
    // this is the complicated (and time-intensive) part: calculate the whole profile from the target to all vertices
    // timestamp is not required in basic version as we're not using corridors
    fn init(&mut self, source: NodeId, target: NodeId, timestamp: u32) {
        // 1. earliest arrival query on lowerbound graph
        let earliest_arrival_distance = self.forward_server.query(Query::new(source, target, 0)).distance().unwrap();

        // initialize backwards profile dijkstra
        // initial time corridor: 3 * lowerbound travel time
        let query = TDPartialBackwardProfileQuery {
            target,
            earliest_arrival: Timestamp(convert_timestamp_u32_to_f64(timestamp + earliest_arrival_distance)),
            initial_timeframe: Timestamp(convert_timestamp_u32_to_f64(3 * earliest_arrival_distance)),
        };
        let mut ops = TDPartialBackwardProfilePotentialOps(&self.travel_time_profile);
        let mut run = DijkstraRun::query(self.backward_graph.borrow(), &mut self.dijkstra, &mut ops, query);

        // run through the whole graph
        let mut counter = 0;
        while let Some(_node) = run.next() {
            counter += 1;
            if counter % 1000 == 0 {
                println!("Finished {} nodes", counter);
            }
        }
        //let (_, time) = measure(|| while let Some(_node) = run.next() {});
        //println!("Potential init took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
    }

    // this is the easy part: lookup at profile of currently inspected node
    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        let timestamp = timestamp % MAX_BUCKETS;

        // get profile of current node
        let profile = &self.dijkstra.distances[node as usize];

        // easy case: profile only contains sentinels -> don't do expensive interpolation
        if profile.len() == 2 {
            Some((1000.0 * profile[0].val.0) as u32)
        } else {
            // get timestamp
            let ts = Timestamp::new(convert_timestamp_u32_to_f64(timestamp));

            // get travel time at `ts`, do something more efficient than PLF evaluation
            let intersect = find_profile_index(profile, ts.0);
            if intersect.is_ok() {
                Some((1000.0 * profile[intersect.unwrap()].val.0) as u32)
            } else {
                // interpolate between next two points
                let intersect = intersect.unwrap_err();
                assert!(intersect > 0 && intersect < profile.len());

                let interpolation_factor = (ts.0 - profile[intersect - 1].at.0) / (profile[intersect].at.0 - profile[intersect - 1].at.0);
                assert!(
                    interpolation_factor >= 0.0 && interpolation_factor <= 1.0,
                    "{:#?}",
                    (interpolation_factor, intersect, &profile)
                );

                Some((((1.0 - interpolation_factor) * profile[intersect - 1].val.0 + interpolation_factor * profile[intersect].val.0) * 1000.0) as u32)
            }
        }
    }
}

/* ------------------------------------------------------------------------------------------- */

pub struct TDPartialBackwardProfilePotentialOps<Profiles>(Profiles);

struct TDPartialBackwardProfileQuery {
    target: NodeId,
    earliest_arrival: Timestamp,
    initial_timeframe: Timestamp,
}

impl GenQuery<Vec<TTFPoint>> for TDPartialBackwardProfileQuery {
    fn new(_from: NodeId, _to: NodeId, _initial_state: Vec<TTFPoint>) -> Self {
        unimplemented!()
    } // not needed

    fn from(&self) -> NodeId {
        self.target
    }

    fn to(&self) -> NodeId {
        unimplemented!()
    }

    fn initial_state(&self) -> Vec<TTFPoint> {
        vec![
            TTFPoint {
                at: self.earliest_arrival,
                val: FlWeight::ZERO,
            },
            TTFPoint {
                at: Timestamp(self.earliest_arrival.0 + self.initial_timeframe.0),
                val: FlWeight::ZERO,
            },
        ]
    }

    fn permutate(&mut self, _order: &NodeOrder) {
        unimplemented!()
    }
}

impl<Profiles: AsRef<Vec<Vec<TTFPoint>>>> DijkstraOps<ReversedGraphWithEdgeIds> for TDPartialBackwardProfilePotentialOps<Profiles> {
    type Label = Vec<TTFPoint>;
    type Arc = (NodeIdT, Reversed);
    type LinkResult = Vec<TTFPoint>;
    type PredecessorLink = (); // no paths are calculated here => not needed

    // label = state at currently processed node
    // must be linked backward with (static) weight at previous edge
    fn link(&mut self, _graph: &ReversedGraphWithEdgeIds, label: &Self::Label, (_, prev_edge): &Self::Arc) -> Self::LinkResult {
        // 1. obtain profile from `previous_node`
        let prev_edge_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(&self.0.as_ref()[prev_edge.0 .0 as usize]));
        let current_node_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(label));

        // 2. link (`prev_profile` and `label`)
        // TODO verify correctness of bounds `start` and `end`
        let upper_bound = prev_edge_profile.static_upper_bound();
        let start = label[0].at - upper_bound;
        let end = (*label.last().unwrap()).at + prev_edge_profile.static_upper_bound();

        let link_result = prev_edge_profile.link(&current_node_profile, start, end);

        return match link_result {
            ATTFContainer::Exact(inner) => inner,
            ATTFContainer::Approx(_, _) => panic!("Result must be exact!"),
        };

        // 3. TODO consider using douglas peuker approximation
    }

    fn merge(&mut self, label: &mut Self::Label, linked: Self::LinkResult) -> bool {
        // easy case: label is empty -> simply uses recently linked profile
        if *label == Self::Label::neutral() {
            *label = linked;
            return true;
        }

        // more complex case requires actual merging
        let linked_profile = PartialPiecewiseLinearFunction::new(&linked);
        let current_profile = PartialPiecewiseLinearFunction::new(label);

        // TODO verify correctness of bounds
        let start = min(
            linked_profile.first().map(|p| p.at).unwrap_or(Timestamp::NEVER),
            current_profile.first().map(|p| p.at).unwrap_or(Timestamp::NEVER),
        );
        let end = max(
            linked_profile.last().map(|p| p.at).unwrap_or(Timestamp::ZERO),
            current_profile.last().map(|p| p.at).unwrap_or(Timestamp::ZERO),
        );
        debug_assert!(start != Timestamp::NEVER && end != Timestamp::ZERO);

        let (result, changes) = current_profile.merge(&linked_profile, start, end, &mut Vec::new());

        // merging takes place if there is any point where the linked profile is 'better',
        // i.e. the current profile does not dominate all the time
        if changes.iter().any(|&(_, b)| !b) {
            *label = result.to_vec();
            true
        } else {
            false
        }
    }

    fn predecessor_link(&self, _link: &Self::Arc) -> Self::PredecessorLink {
        ()
    }
}
