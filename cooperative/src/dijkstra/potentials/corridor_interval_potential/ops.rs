use std::cmp::min;
use rust_road_router::algo::dijkstra::Label;
use rust_road_router::datastr::graph::floating_time_dependent::TTFPoint;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{INFINITY, Reversed, Weight};
use crate::graph::MAX_BUCKETS;

pub struct TDCorridorIntervalPotentialOps {
    pub query_start: Timestamp,
    pub corridor_max: Timestamp,
    pub num_intervals: u32,
    pub edge_intervals: Vec<Vec<Weight>>, // first version: take complete profile -> consider using dynamic arrays here!
}

#[derive(Clone, Debug)]
struct ApproximatedIntervalLabel {
    first_interval_ts: Option<Timestamp>,
    interval_minima: Vec<IntervalLabelEntry>,
}

#[derive(Clone, Debug)]
struct IntervalLabelEntry {
    no_overflow_min: Weight,
    // minimum value extracted without skipping interval bounds
    overflow_min: Option<(Weight, Weight)>, // minimum value extracted with skipping bounds, additional "cooldown"
}

#[derive(Clone, Debug)]
enum IntervalLinkResult {
    WITHOUT_WITHOUT, // no overflows at all, TODO consider renaming
    WITHOUT_WITH(u32), // overflow min of first entry
    WITH_WITHOUT(u32),
    WITH_WITH(u32)
}

impl TDCorridorIntervalPotentialOps {
    pub fn new(
        query_start: Timestamp,
        corridor_max: Timestamp,
        num_intervals: u32,
        departures: &Vec<Vec<Timestamp>>,
        travel_times: &Vec<Vec<Weight>>,
    ) -> Self {
        debug_assert_eq!(departures.len(), travel_times.len());
        debug_assert_eq!(MAX_BUCKETS % interval_length, 0, "All intervals should have the same length!");

        // "customization": iterate over edges, extract minimum travel time within each interval
        // beforehand, we have to determine the interval boundaries
        let interval_length = MAX_BUCKETS / num_intervals;
        let interval_boundaries = (0..num_intervals + 1)
            .into_iter()
            .map(|i| i * interval_length)
            .collect::<Vec<Timestamp>>();

        let edge_intervals: Vec<Vec<Weight>> = departures
            .iter()
            .zip(travel_times.iter())
            .map(|(departure, travel_time)| {
                debug_assert_eq!(departure.len(), travel_time.len());
                extract_interval_minima(departure, travel_time, &interval_boundaries)
            })
            .collect::<Vec<Vec<Weight>>>();

        Self {
            query_start,
            corridor_max,
            num_intervals,
            edge_intervals,
        }
    }

    pub fn link_in_bounds(
        &mut self,
        label: &ApproximatedIntervalLabel,
        prev_edge_id: Reversed,
        node_corridor: (Timestamp, Timestamp),
    ) -> ApproximatedIntervalLabel {
        let mut ret = ApproximatedIntervalLabel {
            first_interval_ts: None,
            interval_minima: Vec::new()
        };

        let prev_edge_id = prev_edge_id.0.0 as usize;

        // compute interval bounds for backward linking (round down in each case)
        let interval_length = MAX_BUCKETS / self.num_intervals;
        let start_interval = interval_length * (node_corridor.0 / interval_length);
        let end_interval = interval_length * (node_corridor.1 / interval_length);

        let mut idx = 0;
        while (start_interval + idx * interval_length) <= end_interval {



            idx += 1;
        }
    }

    pub fn merge_in_bounds(
        &mut self,
        label: &mut ApproximatedIntervalLabel,
        linked: ApproximatedIntervalLabel,
        node_corridor: (Timestamp, Timestamp)
    ) -> bool {
        // easy cases: label or linked result is empty
        if linked.first_interval_ts.is_none() {
            // linking produced no entries -> nothing can be adjusted
            return false;
        } else if label.first_interval_ts.is_none() {
            // first adjustment to node label -> take it over entirely
            *label = linked;
            return true;
        } else {
            // apply slightly more complex merging logic
            let mut changes = false;
            let interval_length = MAX_BUCKETS / self.num_intervals;

            // if labels are incomplete at the start, fill with the smaller value
            let label_start = label.first_interval_ts.unwrap();
            let linked_start = linked.first_interval_ts.unwrap();

            let mut label_iter: dyn Iterator<Item = IntervalLabelEntry> = label.interval_minima.iter();
            let mut linked_iter: dyn Iterator<Item = IntervalLabelEntry> = linked.interval_minima.iter();

            // todo ugly edge case: label and linked are separated intervals

            if label_start < linked_start {
                // nothing needs to be done beside moving the iterator forward
                label_iter.skip(((linked_start - label_start) / interval_length) as usize);
            } else if linked_start < label_start {
                changes = true;
                label.first_interval_ts = linked.first_interval_ts;

                let mut prepended_minima = label_iter
                    .take(((label_start - linked_start) / interval_length) as usize)
                    .collect::<Vec<IntervalLabelEntry>>();

                prepended_minima.extend_from_slice(&label.interval_minima);
                label.interval_minima = prepended_minima;
            }

            // now, label and linked iterators point towards the same timestamp
            // continue to take the next elements until both iterators are exhausted
            while label_iter.f

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
        0*/
    }
}

/// extract the interval minima from a given travel time function
fn extract_interval_minima(departure: &Vec<Timestamp>, travel_time: &Vec<Timestamp>, interval_boundaries: &Vec<Timestamp>) -> Vec<Weight> {
    // 1. compute the minimum of each bucket, existing travel times only (without interpolation)
    let mut interval_buckets = vec![INFINITY; interval_boundaries.len()];
    let mut bucket_idx = 0;

    departure
        .iter()
        .zip(travel_time.iter())
        .for_each(|(&at, &val)| {
            while at < interval_boundaries[bucket_idx + 1] {
                bucket_idx += 1;
                debug_assert!(bucket_idx < interval_boundaries.len() - 1, "sentinel must not be exceeded!");
            }

            interval_buckets[bucket_idx] = min(interval_buckets[bucket_idx], val);
        });

    // 2. combine minima with interpolated values at interval borders
    let ttf = PiecewiseLinearFunction::new(departure, travel_time);

    // latest entry can be ignored (sentinel timestamp of midnight!)
    (0..interval_buckets.len() - 1)
        .into_iter()
        .for_each(|idx| {
            let bucket_start = ttf.eval(interval_boundaries[idx]);
            let bucket_end = ttf.eval(interval_boundaries[idx + 1]);
            interval_buckets[idx] = min(interval_buckets[idx], min(bucket_start, bucket_end));
        });

    interval_buckets
}
