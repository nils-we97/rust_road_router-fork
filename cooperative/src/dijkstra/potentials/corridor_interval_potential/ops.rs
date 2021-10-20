use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::dijkstra::Label;
use rust_road_router::datastr::graph::floating_time_dependent::TTFPoint;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{Reversed, Weight, INFINITY};
use std::cmp::{max, min};

pub struct TDCorridorIntervalPotentialOps {
    pub query_start: Timestamp,
    pub corridor_max: Timestamp,
    pub num_intervals: u32,
    pub interval_length: Timestamp,
    pub edge_intervals: Vec<Vec<Weight>>, // first version: take complete profile -> consider using dynamic arrays here!
}

#[derive(Clone, Debug)]
struct ApproximatedIntervalLabel {
    first_interval_ts: Option<Timestamp>,
    interval_minima: Vec<IntervalLabelEntry>,
}

impl ApproximatedIntervalLabel {
    pub fn new(first_interval_ts: Option<Timestamp>, interval_minima: Vec<IntervalLabelEntry>) -> Self {
        Self {
            first_interval_ts,
            interval_minima,
        }
    }

    pub fn empty() -> Self {
        Self {
            first_interval_ts: None,
            interval_minima: Vec::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct IntervalLabelEntry {
    no_overflow_min: Weight,
    // minimum value extracted without skipping interval bounds
    overflow_min: Option<(Weight, Weight)>, // minimum value extracted with skipping bounds, additional "cooldown"
}

impl IntervalLabelEntry {
    pub fn new(no_overflow_min: Weight, overflow_min: Option<(Weight, Weight)>) -> Self {
        Self { no_overflow_min, overflow_min }
    }
}

#[derive(Clone, Debug)]
enum IntervalLinkResult {
    WITHOUT_WITHOUT,   // no overflows at all, TODO consider renaming
    WITHOUT_WITH(u32), // overflow min of first entry
    WITH_WITHOUT(u32),
    WITH_WITH(u32),
}

impl TDCorridorIntervalPotentialOps {
    pub fn new(query_start: Timestamp, corridor_max: Timestamp, num_intervals: u32, departures: &Vec<Vec<Timestamp>>, travel_times: &Vec<Vec<Weight>>) -> Self {
        debug_assert_eq!(departures.len(), travel_times.len());
        debug_assert_eq!(MAX_BUCKETS % interval_length, 0, "All intervals should have the same length!");

        // "customization": iterate over edges, extract minimum travel time within each interval
        // beforehand, we have to determine the interval boundaries
        let interval_length = MAX_BUCKETS / num_intervals;
        let interval_boundaries = (0..num_intervals + 1).into_iter().map(|i| i * interval_length).collect::<Vec<Timestamp>>();

        let edge_intervals = departures
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
            interval_length: MAX_BUCKETS / num_intervals,
            edge_intervals,
        }
    }

    pub fn link_in_bounds(
        &self,
        label: &ApproximatedIntervalLabel,
        prev_edge_id: Reversed,
        node_corridor: (Timestamp, Timestamp),
    ) -> ApproximatedIntervalLabel {
        let mut ret = ApproximatedIntervalLabel::empty();

        let prev_edge_id = prev_edge_id.0 .0 as usize;

        // compute interval bounds for backward linking (round down in each case)
        let start_interval = self.interval_length * (node_corridor.0 / self.interval_length);
        let end_interval = self.interval_length * (node_corridor.1 / self.interval_length);

        let mut idx = 0;

        while (start_interval + idx * self.interval_length) <= end_interval {
            let current_interval = start_interval + idx * self.interval_length;
            let mut first_entry_found = false;

            // actual linking takes place inside subroutine
            let entry = self.link_internal(label, prev_edge_id, current_interval);

            if let Some(link_entry) = entry {
                ret.interval_minima.push(link_entry);
                if !first_entry_found {
                    first_entry_found = true;
                }
            } else if first_entry_found {
                // TODO verify whether this is correct
                // as soon as we discover the first empty result, we can cancel the linking process
                break;
            }

            idx += 1;
        }

        ret
    }

    fn link_internal(&self, label: &ApproximatedIntervalLabel, prev_edge_id: usize, current_interval: Timestamp) -> Option<IntervalLabelEntry> {
        None
    }

    pub fn merge(&mut self, label: &mut ApproximatedIntervalLabel, linked: ApproximatedIntervalLabel) -> bool {
        // easy cases: label or linked result is empty
        if linked.first_interval_ts.is_none() {
            // linking produced no entries -> nothing can be adjusted
            false
        } else if label.first_interval_ts.is_none() {
            // first adjustment to node label -> take it over entirely
            *label = linked;
            true
        } else {
            // apply slightly more complex merging logic
            let mut changes = false;
            let mut ret = ApproximatedIntervalLabel::empty();

            // if labels are incomplete at the start, fill with the smaller value
            let label_start = label.first_interval_ts.unwrap();
            let linked_start = linked.first_interval_ts.unwrap();

            let mut label_iter = label.interval_minima.iter().peekable();
            let mut linked_iter = linked.interval_minima.iter().peekable();

            // todo ugly edge case: label and linked are separated intervals

            if label_start < linked_start {
                ret.first_interval_ts = Some(label_start);
                ret.interval_minima
                    .extend(label_iter.take(((linked_start - label_start) / self.interval_length) as usize));
            } else if linked_start < label_start {
                changes = true;
                ret.first_interval_ts = Some(linked_start);
                ret.interval_minima
                    .extend(linked_iter.take(((label_start - linked_start) / self.interval_length) as usize));
            }

            // now, label and linked iterators point towards the same timestamp
            // continue to take the next elements until both iterators are exhausted
            loop {
                match (label_iter.next(), linked_iter.next()) {
                    (Some(next_label), Some(next_linked)) => {
                        // push the minimum value of both entries
                        let no_overflow_min = min(next_label.no_overflow_min, next_linked.no_overflow_min);
                        let overflow_min = match (next_label.overflow_min, next_linked.overflow_min) {
                            (Some((label_val, label_counter)), Some((linked_val, linked_counter))) => {
                                // TODO: is this correct?
                                // take the minimum value and its counter as new overflow, it will definitely be smaller than the no-overflow min!
                                // otherwise, this value wouldn't have been passed!
                                if label_val < linked_val {
                                    Some((label_val, label_counter))
                                } else if linked_val < label_val {
                                    Some((linked_val, linked_counter))
                                } else {
                                    Some((label_val, max(label_counter, linked_counter)))
                                }
                            }
                            (Some((label_val, label_counter)), None) => {
                                if label_val < no_overflow_min {
                                    Some((label_val, label_counter))
                                } else {
                                    None
                                }
                            }
                            (None, Some((linked_val, linked_counter))) => {
                                if linked_val < no_overflow_min {
                                    Some((linked_val, linked_counter))
                                } else {
                                    None
                                }
                            }
                            (None, None) => None,
                        };
                        ret.interval_minima.push(IntervalLabelEntry::new(no_overflow_min, overflow_min));
                    }
                    (Some(next_label), None) => {
                        ret.interval_minima.push(next_label.clone());
                    }
                    (None, Some(next_linked)) => {
                        changes = true;
                        ret.interval_minima.push(next_linked.clone());
                    }
                    (None, None) => break,
                }
            }
            changes
        }
    }
}

/// extract the interval minima from a given travel time function
fn extract_interval_minima(departure: &Vec<Timestamp>, travel_time: &Vec<Timestamp>, interval_boundaries: &Vec<Timestamp>) -> Vec<Weight> {
    // 1. compute the minimum of each bucket, existing travel times only (without interpolation)
    let mut interval_buckets = vec![INFINITY; interval_boundaries.len()];
    let mut bucket_idx = 0;

    departure.iter().zip(travel_time.iter()).for_each(|(&at, &val)| {
        while at < interval_boundaries[bucket_idx + 1] {
            bucket_idx += 1;
            debug_assert!(bucket_idx < interval_boundaries.len() - 1, "sentinel must not be exceeded!");
        }

        interval_buckets[bucket_idx] = min(interval_buckets[bucket_idx], val);
    });

    // 2. combine minima with interpolated values at interval borders
    let ttf = PiecewiseLinearFunction::new(departure, travel_time);

    // latest entry can be ignored (sentinel timestamp of midnight!)
    (0..interval_buckets.len() - 1).into_iter().for_each(|idx| {
        let bucket_start = ttf.eval(interval_boundaries[idx]);
        let bucket_end = ttf.eval(interval_boundaries[idx + 1]);
        interval_buckets[idx] = min(interval_buckets[idx], min(bucket_start, bucket_end));
    });

    interval_buckets
}
