use std::borrow::Borrow;

use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, DijkstraRun, Label};
use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, PeriodicPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{BuildReversed, Graph, NodeId, NodeIdT, Reversed, ReversedGraphWithEdgeIds, Weight};
use rust_road_router::datastr::node_order::NodeOrder;

use crate::dijkstra::potentials::{convert_timestamp_u32_to_f64, TDPotential};
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::MAX_BUCKETS;
use crate::util::profile_search::find_profile_index;

/// Basic implementation of a potential obtained by a backward profile search
/// this version is not to be used, but provides a good starting point for further optimizations

pub struct TDBackwardProfilePotential {
    backward_graph: ReversedGraphWithEdgeIds,
    travel_time_profile: Vec<Vec<TTFPoint>>,
    dijkstra: DijkstraData<Vec<TTFPoint>>,
}

impl TDBackwardProfilePotential {
    pub fn new(graph: &CapacityGraph) -> Self {
        let num_nodes = graph.num_nodes();
        let backward_graph = ReversedGraphWithEdgeIds::reversed(graph);

        let departure = graph.departure();
        let travel_time = graph.travel_time();

        let mut ret = Self {
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
                        val: FlWeight::new(convert_timestamp_u32_to_f64(val)),
                    })
                    .collect::<Vec<TTFPoint>>()
            })
            .collect::<Vec<Vec<TTFPoint>>>();
    }
}

impl TDPotential for TDBackwardProfilePotential {
    // this is the complicated (and time-intensive) part: calculate the whole profile from the target to all vertices
    // timestamp is not required in basic version as we're not using corridors
    fn init(&mut self, _source: NodeId, target: NodeId, _timestamp: u32) {
        //initialize backwards profile dijkstra
        let query = TDBackwardProfileQuery(target);
        let mut ops = TDBackwardProfilePotentialOps(&self.travel_time_profile);
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

pub struct TDBackwardProfilePotentialOps<Profiles>(Profiles);

struct TDBackwardProfileQuery(NodeId);

impl GenQuery<Vec<TTFPoint>> for TDBackwardProfileQuery {
    fn new(_from: NodeId, _to: NodeId, _initial_state: Vec<TTFPoint>) -> Self {
        unimplemented!()
    } // not needed

    fn from(&self) -> NodeId {
        self.0
    }

    fn to(&self) -> NodeId {
        unimplemented!()
    }

    fn initial_state(&self) -> Vec<TTFPoint> {
        Vec::<TTFPoint>::neutral()
    }

    fn permutate(&mut self, _order: &NodeOrder) {
        unimplemented!()
    }
}

impl<Profiles: AsRef<Vec<Vec<TTFPoint>>>> DijkstraOps<ReversedGraphWithEdgeIds> for TDBackwardProfilePotentialOps<Profiles> {
    type Label = Vec<TTFPoint>;
    type Arc = (NodeIdT, Reversed);
    type LinkResult = Vec<TTFPoint>;
    type PredecessorLink = (); // no paths are calculated here => not needed

    // label = state at currently processed node
    // must be linked backward with (static) weight at previous edge
    fn link(&mut self, _graph: &ReversedGraphWithEdgeIds, label: &Self::Label, (_, prev_edge): &Self::Arc) -> Self::LinkResult {
        //1. obtain profile from `previous_node`
        let prev_profile = PeriodicPiecewiseLinearFunction::new(&self.0.as_ref()[prev_edge.0 .0 as usize]);
        let current_profile = PeriodicPiecewiseLinearFunction::new(label);

        //2. link (`prev_profile` and `label`)
        let link_result = prev_profile.link(&current_profile);

        PeriodicPiecewiseLinearFunction::new(&link_result).to_vec()

        //3. apply douglas peuker approximation -> accelerates calculation by factor ~10
        // approximation does currently not work! TODO fix this
        //PeriodicPiecewiseLinearFunction::new(&link_result).approximate(&mut Vec::new()).to_vec()
    }

    fn merge(&mut self, label: &mut Self::Label, linked: Self::LinkResult) -> bool {
        // easy case: label is empty -> simply uses recently linked profile
        if *label == Self::Label::neutral() {
            *label = linked;
            return true;
        }

        // more complex case requires actual merging
        let linked_profile = PeriodicPiecewiseLinearFunction::new(&linked);
        let current_profile = PeriodicPiecewiseLinearFunction::new(label);

        let (result, changes) = current_profile.merge(&linked_profile, &mut Vec::new());

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
