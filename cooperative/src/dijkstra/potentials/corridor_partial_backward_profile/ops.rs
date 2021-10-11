use std::cmp::{max, min};
use rust_road_router::datastr::graph::floating_time_dependent::{ATTFContainer, PartialATTF, PartialPiecewiseLinearFunction, Timestamp, TTFPoint};
use rust_road_router::datastr::graph::Reversed;

pub struct TDCorridorPartialBackwardProfilePotentialOps<'a> {
    pub query_start: Timestamp,
    pub corridor_max: Timestamp,
    pub profiles: &'a Vec<Vec<TTFPoint>>,
    pub approximate_threshold: usize,
    pub merge_buffer: Vec<TTFPoint>
}

impl<'a> TDCorridorPartialBackwardProfilePotentialOps<'a> {
    pub fn new(query_start: Timestamp, corridor_max: Timestamp, profiles: &'a Vec<Vec<TTFPoint>>, approximate_threshold: usize) -> Self {
        Self {
            query_start,
            corridor_max,
            profiles,
            approximate_threshold,
            merge_buffer: Vec::new()
        }
    }

    /// approximate a given label if the number of breakpoints exceeds `threshold`
    pub fn approximate_at_threshold(&self, label: &mut Vec<TTFPoint>) {
        if label.len() > self.approximate_threshold {
            *label = PartialPiecewiseLinearFunction::new(label)
                .lower_bound_ttf(&mut Vec::new(), &mut Vec::new())
                .to_vec();
        }
    }

    pub fn link_in_bounds(&mut self, label: &Vec<TTFPoint>, prev_edge_id: Reversed, node_corridor: (Timestamp, Timestamp)) -> Vec<TTFPoint> {
        // 1. obtain profile for previous edge, expand it if needed
        let prev_edge_ipps = &self.profiles[prev_edge_id.0 .0 as usize];
        let extended_profile = extend_edge_profile(prev_edge_ipps, label.last().unwrap().at);
        let prev_edge_ipps = extended_profile.as_ref().unwrap_or(prev_edge_ipps);

        // 2. obtain start/end timestamp by corridor, double check for bounds
        let ts_start = max(self.query_start, node_corridor.0);
        let ts_end = min(self.corridor_max, node_corridor.1);

        if ts_start.fuzzy_lt(ts_end) {
            // 3. link the previous edge profile with the current node profile
            let prev_edge_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(prev_edge_ipps));
            let current_node_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(label));

            let link_result = prev_edge_profile.link(&current_node_profile, ts_start, ts_end);

            match link_result {
                ATTFContainer::Exact(inner) => { inner }
                ATTFContainer::Approx(_, _) => panic!("Result must be exact, this should not happen!"),
            }
        } else {
            // this edge is not relevant!
            // Even for the latest relevant arrival at this edge, the start takes place before the query's departure
            Vec::new()
        }
    }

    pub fn merge_in_bounds(&mut self, label: &mut Vec<TTFPoint>, mut linked: Vec<TTFPoint>, node_corridor: (Timestamp, Timestamp)) -> bool {
        // easy cases: label or linked result is empty
        if linked.is_empty() {
            // no linking happened, so we can simply take the previous result
            return false;
        } else if label.is_empty() {
            // first non-empty adjustment, so we can simply take the new linked PLF
            *label = linked;
            return true;
        }

        // assure that both profiles can be merged within the given bounds
        debug_assert!(label.first().unwrap().at.fuzzy_leq(node_corridor.0));
        debug_assert!(linked.first().unwrap().at.fuzzy_leq(node_corridor.0));
        debug_assert!(node_corridor.1.fuzzy_leq(label.last().unwrap().at));
        debug_assert!(node_corridor.1.fuzzy_leq(linked.last().unwrap().at));

        let linked_profile = PartialPiecewiseLinearFunction::new(&linked);
        let current_profile = PartialPiecewiseLinearFunction::new(label);

        let (result, changes) = current_profile.merge(&linked_profile, node_corridor.0, node_corridor.1, &mut self.merge_buffer);

        // merging takes place if there is any point where the linked profile is 'better',
        // i.e. the current profile does not dominate all the time
        // even if the current profile dominates all the time, we might have adjusted the time bounds!
        if changes.iter().any(|&(_, b)| !b) {
            *label = result.to_vec();
            true
        } else {
            false
        }
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
