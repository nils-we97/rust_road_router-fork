use rust_road_router::algo::dijkstra::{DijkstraOps, Label};
use rust_road_router::datastr::graph::floating_time_dependent::{ATTFContainer, FlWeight, PartialATTF, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::{NodeIdT, Reversed, ReversedGraphWithEdgeIds};
use std::cmp::{max, min};

/// extend label of `Vec<TTFPoint>` by saving the lowerbound distance
#[derive(Clone, Debug, PartialEq)]
pub struct DirectedPartialBackwardProfileLabel {
    pub ttf: Vec<TTFPoint>, // careful: partial ttf includes interval [ttf_start, ttf_end]
    pub ttf_start: Timestamp,
    pub ttf_end: Timestamp,
    pub min_dist: FlWeight,
}

impl Label for DirectedPartialBackwardProfileLabel {
    type Key = FlWeight;

    fn neutral() -> Self {
        Self {
            ttf: Vec::<TTFPoint>::neutral(),
            ttf_start: Timestamp::neutral(),
            ttf_end: Timestamp::neutral(),
            min_dist: FlWeight::ZERO,
        }
    }

    fn key(&self) -> Self::Key {
        self.ttf.key()
    }
}

pub struct TDDirectedPartialBackwardProfilePotentialOps<Profiles> {
    pub query_start: Timestamp,
    pub corridor_max: Timestamp,
    pub profiles: Profiles,
    pub approximate_threshold: usize,
}

impl<Profiles: AsRef<Vec<Vec<TTFPoint>>>> TDDirectedPartialBackwardProfilePotentialOps<Profiles> {
    pub fn new(query_start: Timestamp, corridor_max: Timestamp, profiles: Profiles, approximate_threshold: usize) -> Self {
        Self {
            query_start,
            corridor_max,
            profiles,
            approximate_threshold,
        }
    }

    /// approximate a given label if the number of breakpoints exceeds `threshold`
    pub fn approximate_at_threshold(&self, label: &mut DirectedPartialBackwardProfileLabel) {
        if label.ttf.len() > self.approximate_threshold {
            label.ttf = PartialPiecewiseLinearFunction::new(&label.ttf)
                .lower_bound_ttf(&mut Vec::new(), &mut Vec::new())
                .to_vec();
            label.min_dist = label.ttf.iter().map(|p| p.val).min().unwrap();
        }
    }
}

impl<Profiles: AsRef<Vec<Vec<TTFPoint>>>> DijkstraOps<ReversedGraphWithEdgeIds> for TDDirectedPartialBackwardProfilePotentialOps<Profiles> {
    type Label = DirectedPartialBackwardProfileLabel;
    type Arc = (NodeIdT, Reversed);
    type LinkResult = DirectedPartialBackwardProfileLabel;
    type PredecessorLink = (); // no paths are calculated here => not needed

    // label = state at currently processed node
    // must be linked backward with (static) weight at previous edge
    fn link(&mut self, _graph: &ReversedGraphWithEdgeIds, label: &Self::Label, (_, prev_edge_id): &Self::Arc) -> Self::LinkResult {
        // 1. obtain profile for previous edge, expand it if needed
        let prev_edge_ipps = &self.profiles.as_ref()[prev_edge_id.0 .0 as usize];
        let extended_profile = extend_edge_profile(prev_edge_ipps, label.ttf.last().unwrap().at);
        let prev_edge_ipps = extended_profile.as_ref().unwrap_or(prev_edge_ipps);

        // 2. obtain start/end timestamp for linking
        // use earliest-arrival-query information to limit the corridor!
        let backward_start = backward_link_interval(prev_edge_ipps, label.ttf_start);
        let ts_start = max(self.query_start, backward_start.unwrap_or(Timestamp::ZERO));

        let backward_end = backward_link_interval(prev_edge_ipps, label.ttf_end);
        let ts_end = min(self.corridor_max, backward_end.unwrap_or(Timestamp::NEVER));

        if ts_start.fuzzy_lt(ts_end) && backward_end.is_some() {
            // 3. link the previous edge profile with the current node profile
            let prev_edge_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(prev_edge_ipps));
            let current_node_profile = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(&label.ttf));

            let link_result = prev_edge_profile.link(&current_node_profile, ts_start, ts_end);

            match link_result {
                ATTFContainer::Exact(inner) => {
                    let min_dist = inner.iter().map(|p| p.val).min().unwrap();
                    Self::LinkResult {
                        ttf: inner,
                        ttf_start: ts_start,
                        ttf_end: ts_end,
                        min_dist,
                    }
                }
                ATTFContainer::Approx(_, _) => panic!("Result must be exact, this should not happen!"),
            }
        } else {
            // this edge is not relevant!
            // Even for the latest relevant arrival at this edge, the start takes place before the query's departure
            Self::LinkResult::neutral()
        }
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

        debug_assert!(self.query_start.fuzzy_leq(linked.ttf_start));
        debug_assert!(self.query_start.fuzzy_leq(label.ttf_start));
        debug_assert!(linked.ttf_end.fuzzy_leq(self.corridor_max));
        debug_assert!(label.ttf_end.fuzzy_leq(self.corridor_max));

        // more complex case requires actual merging
        // first of all, adjust labels to span the same range (by adding fifo-conform breakpoints at start/end of shorter profile)
        let label_adjusted = adjust_labels_to_merge(label, &mut linked);

        // with the same start & end timestamp, we can just take the merge result
        let linked_profile = PartialPiecewiseLinearFunction::new(&linked.ttf);
        let current_profile = PartialPiecewiseLinearFunction::new(&label.ttf);

        let (result, changes) = current_profile.merge(&linked_profile, label.ttf_start, label.ttf_end, &mut Vec::new());

        // merging takes place if there is any point where the linked profile is 'better',
        // i.e. the current profile does not dominate all the time
        // even if the current profile dominates all the time, we might have adjusted the time bounds!
        if label_adjusted || significant_changes(&changes, &linked.ttf, &label.ttf) {
            label.ttf = result.to_vec();
            label.min_dist = min(label.min_dist, linked.min_dist);
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

/// adjust the labels to span the same interval w.r.t. the valid start/end of the TTFs
/// returns true if adjustment to the existing label were made
fn adjust_labels_to_merge(label: &mut DirectedPartialBackwardProfileLabel, linked: &mut DirectedPartialBackwardProfileLabel) -> bool {
    // in order to merge within bounds, we first assure that each PLF spans the same interval
    // as soon as we have to extend `label`, we can assume that changes will be made
    let mut label_adjusted = false;

    // extend functions to the same start timestamp
    if label.ttf_start.fuzzy_lt(linked.ttf[0].at) {
        // add FIFO-conform weight at beginning, equal to "waiting"
        // time difference is (linked[0].at - label.ttf_start),
        // hence linked gets a new starting point at label.ttf_start with additional waiting time equal to the time difference
        linked.ttf.insert(
            0,
            TTFPoint {
                at: label.ttf_start,
                val: linked.ttf[0].val + linked.ttf[0].at - label.ttf_start,
            },
        );
        linked.ttf_start = label.ttf_start;
    } else if linked.ttf_start.fuzzy_lt(label.ttf[0].at) {
        label.ttf.insert(
            0,
            TTFPoint {
                at: linked.ttf_start,
                val: label.ttf[0].val + label.ttf[0].at - linked.ttf_start,
            },
        );
        label.ttf_start = linked.ttf_start;
        label_adjusted = true;
    }

    let label_last = label.ttf.last().unwrap().clone();
    let linked_last = linked.ttf.last().unwrap().clone();

    if label_last.at.fuzzy_lt(linked.ttf_end) {
        label.ttf.push(TTFPoint {
            at: linked.ttf_end,
            val: label_last.val + linked.ttf_end - label_last.at,
        });
        label.ttf_end = linked.ttf_end;
        label_adjusted = true;
    } else if linked_last.at.fuzzy_lt(label.ttf_end) {
        linked.ttf.push(TTFPoint {
            at: label.ttf_end,
            val: linked_last.val + label.ttf_end - linked_last.at,
        });
        linked.ttf_end = label.ttf_end;
    }

    label_adjusted
}

/// checks whether the changes occurred during merging are significant
fn significant_changes(changes: &Vec<(Timestamp, bool)>, linked: &Vec<TTFPoint>, label: &Vec<TTFPoint>) -> bool {
    // revoke the changes if the effect is negligible
    // first of all, check where the current label is dominated by the linked profile
    let improving_changes_ts = changes.iter().filter(|&&(_, b)| !b).map(|&(ts, _)| ts).collect::<Vec<Timestamp>>();

    let linked_profile = PartialPiecewiseLinearFunction::new(linked);
    let label_profile = PartialPiecewiseLinearFunction::new(label);

    if improving_changes_ts.is_empty() {
        // old label dominates all the time -> no changes!
        false
    } else if improving_changes_ts.len() > 0 && improving_changes_ts.len() < 10 {
        // few changes, check if they are relevant or not
        let total_changes: f64 = improving_changes_ts
            .iter()
            .map(|&ts| (linked_profile.eval(ts) - label_profile.eval(ts)).abs().0)
            .sum();

        total_changes < 0.00001
    } else {
        // too many changes, for now we assume at least one change is significant
        true
    }
}
