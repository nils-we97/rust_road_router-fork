use std::borrow::Borrow;
use std::cmp::{max, min};

use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, DijkstraRun, Label, Server};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::floating_time_dependent::{ATTFContainer, FlWeight, PartialATTF, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{BuildReversed, EdgeId, FirstOutGraph, Graph, NodeId, NodeIdT, Reversed, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::node_order::NodeOrder;

use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;

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

pub struct TDPartialBackwardProfilePotentialOps<Profiles> {
    pub query_start: Timestamp,
    pub corridor_max: Timestamp,
    pub profiles: Profiles,
}

pub struct TDPartialBackwardProfileQuery {
    pub target: NodeId,
    pub earliest_arrival: Timestamp,
    pub initial_timeframe: Timestamp,
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
        // 1. obtain profile for previous edge, expand it if needed
        let prev_edge_ipps = &self.profiles.as_ref()[prev_edge.0 .0 as usize];
        let extended_profile = extend_edge_profile(prev_edge_ipps, label.last().unwrap().at);
        let prev_edge_ipps = extended_profile.as_ref().unwrap_or(prev_edge_ipps);

        // 2. obtain start/end timestamp for linking
        // use earliest-arrival-query information to limit the corridor!
        let backward_start = backward_link_interval(prev_edge_ipps, label[0].at);
        let ts_start = max(self.query_start, backward_start.unwrap_or(Timestamp::ZERO));

        let backward_end = backward_link_interval(prev_edge_ipps, label.last().unwrap().at);
        let ts_end = min(self.corridor_max, backward_end.unwrap_or(Timestamp::NEVER));

        if ts_start.fuzzy_lt(ts_end) && backward_end.is_some() {
            // 3. link the previous edge profile with the current node profile
            let prev_edge_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(prev_edge_ipps));
            let current_node_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(label));

            let link_result = prev_edge_profile.link(&current_node_profile, ts_start, ts_end);

            match link_result {
                ATTFContainer::Exact(inner) => inner,
                ATTFContainer::Approx(_, _) => panic!("Result must be exact, this should not happen!"),
            }
        } else {
            // this edge is not relevant!
            // Even for the latest relevant arrival at this edge, the start takes place before the query's departure
            Self::LinkResult::neutral()
        }

        // 4. TODO consider using some approximation
    }

    fn merge(&mut self, label: &mut Self::Label, mut linked: Self::LinkResult) -> bool {
        // easy cases: label or linked result is empty
        if linked == Self::LinkResult::neutral() {
            // no linking happened, so we can simply take the previous result
            return false;
        } else if *label == Self::Label::neutral() {
            // first non-empty adjustment, so we can simply take the new linked PLF
            *label = linked;
            return true;
        }

        // more complex case requires actual merging
        // first of all, adjust labels to span the same range (by adding fifo-conform breakpoints at start/end of shorter profile)
        let (start_ts, end_ts, label_adjusted) = adjust_labels_to_merge(label, &mut linked);

        // with the same start & end timestamp, we can just take the merge result
        let linked_profile = PartialPiecewiseLinearFunction::new(&linked);
        let current_profile = PartialPiecewiseLinearFunction::new(label);

        let (result, changes) = current_profile.merge(&linked_profile, start_ts, end_ts, &mut Vec::new());

        // merging takes place if there is any point where the linked profile is 'better',
        // i.e. the current profile does not dominate all the time
        // even if the current profile dominates all the time, we might have adjusted the time bounds!
        if label_adjusted || changes.iter().any(|&(_, b)| !b) {
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

// helper functions

/// finds the timestamp in an (extended) edge profile needed for backward linking
fn backward_link_interval(prev_edge_ipps: &Vec<TTFPoint>, reference_ts: Timestamp) -> Option<Timestamp> {
    // find profile breakpoints in relevant interval
    // as the profile is already extended, we do not have to care about clipping the timestamp to [0, 86400.0]
    let idx_start = prev_edge_ipps.binary_search_by_key(&reference_ts, |p| p.at + p.val);
    if idx_start.is_ok() {
        // easy, but very rare case: previous edge contains TTF point that directly links with start of current profile
        Some(prev_edge_ipps[idx_start.unwrap()].at)
    } else if idx_start.unwrap_err() == 0 {
        // no backward link found: if the element would have to be inserted at the first slot,
        // it is impossible to reach the reference timestamp when starting at this edge (even at midnight!)
        None
    } else {
        // more complex, general case: interpolate between the two point that are in between the desired solution
        // goal: for interpolation function f, find timestamp t such that: t + f(t) = reference_ts
        let ipp1 = &prev_edge_ipps[idx_start.unwrap_err() - 1];
        let ipp2 = &prev_edge_ipps[idx_start.unwrap_err()];

        // fortunately, there is a closed solution formula to obtain the desired point
        // get slope of TTF interval
        let delta = (ipp2.val.0 - ipp1.val.0) / (ipp2.at.0 - ipp1.at.0);
        debug_assert_ne!(delta, -1.0, "slope must not be -1 here!");

        Some(Timestamp((reference_ts.0 - ipp1.val.0 + ipp1.at.0 * delta) / (1.0 + delta)))
    }
}

/// extends a given edge profile over midnight to cover at least `last_required_ts`
fn extend_edge_profile(profile: &Vec<TTFPoint>, last_required_ts: Timestamp) -> Option<Vec<TTFPoint>> {
    if last_required_ts.fuzzy_lt(Timestamp(86400.0)) {
        None
    } else {
        // reserve more space than needed to avoid re-allocs
        // TODO consider making this more efficient by removing unnecessary elements at the beginning!
        let num_rounds = (last_required_ts.0 / 86400.0) as usize + 1;
        let mut ret = Vec::with_capacity(num_rounds * profile.len());

        ret.extend_from_slice(profile);

        // extend `ret` until `last_required_ts` is covered
        let mut current_round = 86400.0;
        let mut current_profile_idx = 1; // ignore the first entry because it is equal to the sentinel at the end!

        while ret.last().unwrap().at < last_required_ts {
            ret.push(TTFPoint {
                at: Timestamp(current_round + ret[current_profile_idx].at.0),
                val: ret[current_profile_idx].val,
            });
            current_profile_idx += 1;
            if current_profile_idx == profile.len() {
                current_profile_idx = 1;
                current_round += 86400.0;
            }
        }

        Some(ret)
    }
}

/// adjust the labels to span the same interval
/// additional return values: (common start timestamp, common end time stamp, adjustments to `label`)
fn adjust_labels_to_merge(label: &mut Vec<TTFPoint>, linked: &mut Vec<TTFPoint>) -> (Timestamp, Timestamp, bool) {
    // TODO verify whether the following assumption is correct
    // in order to merge within bounds, we first assure that each PLF spans the same interval
    // as soon as we have to extend `label`, we can assume that changes will be made
    let mut label_adjusted = false;

    // extend functions to the same start timestamp
    if label[0].at.fuzzy_lt(linked[0].at) {
        // add FIFO-conform weight at beginning, equal to "waiting"
        // time difference is (linked[0].at - label[0].at),
        // hence linked gets a new starting point at label[0].at with additional waiting time equal to the time difference
        linked.insert(
            0,
            TTFPoint {
                at: label[0].at,
                val: linked[0].val + linked[0].at - label[0].at,
            },
        );
    } else if linked[0].at.fuzzy_lt(label[0].at) {
        label.insert(
            0,
            TTFPoint {
                at: linked[0].at,
                val: label[0].val + label[0].at - linked[0].at,
            },
        );
        label_adjusted = true;
    }

    // TODO is it necessary to extend functions to the same end timestamp?
    let label_last = label.last().unwrap().clone();
    let linked_last = linked.last().unwrap().clone();

    if label_last.at.fuzzy_lt(linked_last.at) {
        label.push(TTFPoint {
            at: linked_last.at,
            val: label_last.val + linked_last.at - label_last.at,
        });
        label_adjusted = true;
    } else if linked_last.at.fuzzy_lt(label_last.at) {
        linked.push(TTFPoint {
            at: label_last.at,
            val: linked_last.val + label_last.at - linked_last.at,
        });
    }

    (label.first().unwrap().at, label.last().unwrap().at, label_adjusted)
}
