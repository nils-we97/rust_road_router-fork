use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::dijkstra::Label;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{Reversed, Weight, INFINITY};
use std::cmp::{max, min};

#[derive(Clone, Debug)]
pub struct TDCorridorIntervalPotentialOps {
    pub num_intervals: u32,
    pub interval_length: Timestamp,
    pub edge_intervals: Vec<Vec<Weight>>, // first version: take complete profile -> consider using dynamic arrays here!
    reset_threshold: Timestamp,
}

#[derive(Clone, Debug)]
pub struct ApproximatedIntervalLabel {
    pub first_interval_ts: Option<Timestamp>,
    pub interval_minima: Vec<IntervalLabelEntry>,
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

    pub fn max_dist(&self) -> Weight {
        // exploit the circumstance that the overflow min value is always smaller than the no-overflows!
        self.interval_minima
            .iter()
            .max_by_key(|entry| entry.no_overflow_min)
            .map(|entry| entry.no_overflow_min)
            .unwrap_or(INFINITY)
    }
}

impl Label for ApproximatedIntervalLabel {
    type Key = Weight;

    fn neutral() -> Self {
        ApproximatedIntervalLabel::empty()
    }

    fn key(&self) -> Self::Key {
        self.interval_minima
            .iter()
            .map(|entry| entry.overflow_min.map(|(val, _)| val).unwrap_or(entry.no_overflow_min))
            .min()
            .unwrap_or(INFINITY)
    }
}

#[derive(Clone, Debug)]
pub struct IntervalLabelEntry {
    pub no_overflow_min: Weight,
    // minimum value extracted without skipping interval bounds
    pub overflow_min: Option<(Weight, Weight)>, // minimum value extracted with skipping bounds, additional "cooldown"
}

impl IntervalLabelEntry {
    pub fn new(no_overflow_min: Weight, overflow_min: Option<(Weight, Weight)>) -> Self {
        Self { no_overflow_min, overflow_min }
    }
}

#[derive(Clone, Debug)]
enum IntervalLinkResult {
    BeginningWithoutOverflow(u32),   // first interval value without overflow is the best one
    BeginningWithOverflow(u32, u32), // overflow value is the best -> also update reset counter
    EndWithoutOverflow(u32),         // second interval value without overflow wins
    EndWithOverflow(u32),            // reset counter can be omitted, is 0 anyway
    Empty,
}

impl TDCorridorIntervalPotentialOps {
    pub fn new(num_intervals: u32, departures: &Vec<Vec<Timestamp>>, travel_times: &Vec<Vec<Weight>>) -> Self {
        let interval_length = MAX_BUCKETS / num_intervals;
        let interval_boundaries = (0..num_intervals + 1).into_iter().map(|i| i * interval_length).collect::<Vec<Timestamp>>();

        debug_assert_eq!(departures.len(), travel_times.len());
        debug_assert_eq!(MAX_BUCKETS % interval_length, 0, "All intervals should have the same length!");

        // "customization": iterate over edges, extract minimum travel time within each interval
        // beforehand, we have to determine the interval boundaries

        let edge_intervals = departures
            .iter()
            .zip(travel_times.iter())
            .map(|(departure, travel_time)| {
                debug_assert_eq!(departure.len(), travel_time.len());
                extract_interval_minima(departure, travel_time, &interval_boundaries)
            })
            .collect::<Vec<Vec<Weight>>>();

        Self {
            num_intervals,
            interval_length: MAX_BUCKETS / num_intervals,
            edge_intervals,
            reset_threshold: MAX_BUCKETS / num_intervals,
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
        let mut first_entry_found = false;

        while (start_interval + idx * self.interval_length) <= end_interval {
            let current_interval_ts = start_interval + idx * self.interval_length;

            // actual linking takes place inside subroutine
            let entry = self.link_internal(label, prev_edge_id, current_interval_ts, node_corridor);

            if let Some(link_entry) = entry {
                ret.interval_minima.push(link_entry);
                if !first_entry_found {
                    first_entry_found = true;
                    ret.first_interval_ts = Some(current_interval_ts);
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

    fn link_internal(
        &self,
        label: &ApproximatedIntervalLabel,
        prev_edge_id: usize,
        timestamp: Timestamp,
        node_corridor: (Timestamp, Timestamp),
    ) -> Option<IntervalLabelEntry> {
        // 1. get travel time along edge within the current interval
        let edge_profile_index = ((timestamp % MAX_BUCKETS) / self.interval_length) as usize;
        let edge_tt: Weight = self.edge_intervals[prev_edge_id][edge_profile_index];

        // 2. get relevant values from the current label
        // use direct indexing: earliest arrival time possible when departing at `timestamp`, restrict to valid entries
        // also consider possible subtraction overflows + special case treatment for first and last element

        // special case treatment for first element: check whether the first interval is really necessary,
        // it might happen that corridor_lower + edge_tt are actually falling into the next interval
        let label_index_at_begin = Some(max(timestamp, node_corridor.0) + edge_tt)
            .filter(|&val| val > label.first_interval_ts.unwrap())
            .map(|val| ((val - label.first_interval_ts.unwrap()) / self.interval_length) as usize);
        let entry_at_begin: Option<&IntervalLabelEntry> = label_index_at_begin.and_then(|i| label.interval_minima.get(i));

        // special case treatment for the last element: check whether the overflow interval is really necessary,
        // it might happen that it will never be reached
        let second_ts = if node_corridor.1 - timestamp < self.interval_length {
            node_corridor.1
        } else {
            timestamp + self.interval_length
        };
        let label_index_at_end = Some(second_ts + edge_tt)
            .filter(|&val| val > label.first_interval_ts.unwrap())
            .map(|val| ((val - label.first_interval_ts.unwrap()) / self.interval_length) as usize);
        let entry_at_end: Option<&IntervalLabelEntry> = label_index_at_end.and_then(|i| label.interval_minima.get(i));

        // TODO verify the following assumption:
        // it suffices to compare the entry at `label_index` and `label_index + 1` as the travel time is considered constant in each interval

        // 3. run subroutine to determine which value (and overflow) to use for linking
        let min_link_val = get_minimum_link_value(entry_at_begin, entry_at_end);

        // get the minimum value without any overflows, perform additional corridor checks
        let no_overflow_min = match (entry_at_begin, entry_at_end) {
            (Some(entry_begin), Some(_)) if label_index_at_begin.unwrap() <= label_index_at_end.unwrap() => edge_tt + entry_begin.no_overflow_min,
            (Some(entry_begin), None) => edge_tt + entry_begin.no_overflow_min,
            (None, Some(entry_end)) => edge_tt + entry_end.no_overflow_min,
            _ => {
                // covers (None, None) and panics if idx_begin > idx_end!
                debug_assert!(entry_at_begin.is_none() && entry_at_end.is_none());
                INFINITY
            }
        };

        // 4. perform linking based on the minimum link value
        let result = match min_link_val {
            IntervalLinkResult::Empty => None,
            IntervalLinkResult::BeginningWithoutOverflow(val) => Some(IntervalLabelEntry::new(edge_tt + val, None)),
            IntervalLinkResult::BeginningWithOverflow(val, reset_counter) => {
                // modify reset counter, reset to 0 if it exceeds the interval length (TODO verify)
                let updated_reset_counter = if reset_counter + edge_tt >= self.reset_threshold {
                    0
                } else {
                    reset_counter + edge_tt
                };

                // also store the overflow-free value (necessary for later linking operations with overflows)
                Some(IntervalLabelEntry::new(
                    no_overflow_min,
                    if edge_tt + val < no_overflow_min {
                        Some((edge_tt + val, updated_reset_counter))
                    } else {
                        None
                    },
                ))
            }
            IntervalLinkResult::EndWithoutOverflow(val) => {
                // take the new `val` as reset counter as overflows haven't been applied to it yet
                let reset_counter = if val >= self.reset_threshold { 0 } else { val };
                Some(IntervalLabelEntry::new(
                    no_overflow_min,
                    if edge_tt + val < no_overflow_min {
                        Some((edge_tt + val, reset_counter))
                    } else {
                        None
                    },
                ))
            }
            IntervalLinkResult::EndWithOverflow(val) => {
                // increase the reset counter by edge traversal time
                // if we take the whole distance here, we propagate the lowerbound distances too fast
                let reset_counter = if edge_tt >= self.reset_threshold { 0 } else { edge_tt };

                Some(IntervalLabelEntry::new(
                    no_overflow_min,
                    if edge_tt + val < no_overflow_min {
                        Some((edge_tt + val, reset_counter))
                    } else {
                        None
                    },
                ))
            }
        };
        result
    }

    pub fn merge(&self, label: &mut ApproximatedIntervalLabel, linked: ApproximatedIntervalLabel) -> bool {
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

            let label_start = label.first_interval_ts.unwrap();
            let linked_start = linked.first_interval_ts.unwrap();

            // if one of the labels starts earlier, use its values until we have comparable entries
            let (mut label_iter, mut linked_iter) = if label_start < linked_start {
                let idx = ((linked_start - label_start) / self.interval_length) as usize;
                ret.first_interval_ts = Some(label_start);
                ret.interval_minima.extend_from_slice(&label.interval_minima[..idx]);

                (label.interval_minima[idx..].iter(), linked.interval_minima.iter())
            } else if linked_start == label_start {
                ret.first_interval_ts = Some(label_start);
                (label.interval_minima.iter(), linked.interval_minima.iter())
            } else {
                let idx = ((label_start - linked_start) / self.interval_length) as usize;

                ret.first_interval_ts = Some(linked_start);
                ret.interval_minima.extend_from_slice(&linked.interval_minima[..idx]);
                changes = true;

                (label.interval_minima.iter(), linked.interval_minima[idx..].iter())
            };

            // now, label and linked iterators point towards the same timestamp
            // continue to take the next elements until both iterators are exhausted
            loop {
                match (label_iter.next(), linked_iter.next()) {
                    (Some(next_label), Some(next_linked)) => {
                        // push the minimum value of both entries
                        let no_overflow_min = if next_linked.no_overflow_min < next_label.no_overflow_min {
                            changes = true;
                            next_linked.no_overflow_min
                        } else {
                            next_label.no_overflow_min
                        };

                        let overflow_min = match (next_label.overflow_min, next_linked.overflow_min) {
                            (Some((label_val, label_counter)), Some((linked_val, linked_counter))) => {
                                // TODO: is this correct?
                                // take the minimum value and its counter as new overflow, it will definitely be smaller than the no-overflow min!
                                // otherwise, this value wouldn't have been passed!
                                if label_val < linked_val {
                                    Some((label_val, label_counter))
                                } else if linked_val < label_val {
                                    changes = true;
                                    Some((linked_val, linked_counter))
                                } else if label_counter < linked_counter {
                                    changes = true;
                                    Some((label_val, linked_counter))
                                } else {
                                    Some((label_val, label_counter))
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
                                    changes = true;
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
            *label = ret;
            changes
        }
    }
}

/// extract the interval minima from a given travel time function
fn extract_interval_minima(departure: &Vec<Timestamp>, travel_time: &Vec<Timestamp>, interval_boundaries: &Vec<Timestamp>) -> Vec<Weight> {
    // 1. compute the minimum of each bucket, existing travel times only (without interpolation)
    let mut interval_buckets = vec![INFINITY; interval_boundaries.len() - 1];
    let mut bucket_idx = 0;

    departure[..departure.len() - 1]
        .iter()
        .zip(travel_time[..travel_time.len() - 1].iter())
        .for_each(|(&at, &val)| {
            while at >= interval_boundaries[bucket_idx + 1] {
                bucket_idx += 1;
                debug_assert!(bucket_idx < interval_boundaries.len() - 1, "sentinel must not be exceeded!");
            }

            interval_buckets[bucket_idx] = min(interval_buckets[bucket_idx], val);
        });

    // 2. combine minima with interpolated values at interval borders
    let ttf = PiecewiseLinearFunction::new(departure, travel_time);

    // latest entry can be ignored (sentinel timestamp of midnight!)
    (0..interval_buckets.len()).into_iter().for_each(|idx| {
        let bucket_start = ttf.eval(interval_boundaries[idx]);
        let bucket_end = ttf.eval(interval_boundaries[idx + 1]);
        interval_buckets[idx] = min(interval_buckets[idx], min(bucket_start, bucket_end));
    });

    interval_buckets
}

/// subroutine: extracts the minimum feasible value to link along the current edge at the selected timestamp
fn get_minimum_link_value(entry_at_begin: Option<&IntervalLabelEntry>, entry_at_end: Option<&IntervalLabelEntry>) -> IntervalLinkResult {
    match (entry_at_begin, entry_at_end) {
        (Some(begin), Some(end)) => {
            // most common case, both entries exist
            // use overflow values if they exist, but carefully: overflow of end may not be used if not resetted!
            // otherwise, we would quickly propagate the lowerbound distances through all intervals and thus making the whole procedure useless!

            // check which values to compare, use no-overflow values wherever possible!
            match (begin.overflow_min, end.overflow_min) {
                (Some((begin_overflow_val, begin_overflow_reset)), Some((end_overflow_val, end_overflow_reset))) => {
                    // overflow values are the best in both intervals
                    // also check whether the second value may be used, i.e. the reset-counter is 0!
                    // tie-breaking: use the overflow value at the begin (TODO verify)
                    if end_overflow_reset == 0 && end_overflow_val < begin_overflow_val {
                        IntervalLinkResult::EndWithOverflow(end_overflow_val)
                    } else {
                        IntervalLinkResult::BeginningWithOverflow(begin_overflow_val, begin_overflow_reset)
                    }
                }
                (Some((begin_overflow_val, begin_overflow_reset)), None) => {
                    // overflow val is the best for the first interval only
                    // both values can be safely used!
                    // tie-breaking: use the overflow value and increase reset counter for next label!

                    if end.no_overflow_min < begin_overflow_val {
                        IntervalLinkResult::EndWithoutOverflow(end.no_overflow_min)
                    } else {
                        IntervalLinkResult::BeginningWithOverflow(begin_overflow_val, begin_overflow_reset)
                    }
                }
                (None, Some((end_overflow_val, end_overflow_reset))) => {
                    // overflow val is the best for the second interval only
                    // also check whether the value may be used!

                    if end_overflow_reset == 0 && end_overflow_val < begin.no_overflow_min {
                        IntervalLinkResult::EndWithOverflow(end_overflow_val)
                    } else {
                        IntervalLinkResult::BeginningWithoutOverflow(begin.no_overflow_min)
                    }
                }
                (None, None) => {
                    if end.no_overflow_min < begin.no_overflow_min {
                        IntervalLinkResult::EndWithoutOverflow(end.no_overflow_min)
                    } else {
                        IntervalLinkResult::BeginningWithoutOverflow(begin.no_overflow_min)
                    }
                }
            }
        }
        (Some(begin), None) => {
            // only the first entry exists -> take the overflow min, if it exists it is the best option!
            if let Some((val, reset_counter)) = begin.overflow_min {
                IntervalLinkResult::BeginningWithOverflow(val, reset_counter)
            } else {
                IntervalLinkResult::BeginningWithoutOverflow(begin.no_overflow_min)
            }
        }
        (None, Some(end)) => {
            // only the last entry exists -> check whether the overflow can be used or not
            if let Some((val, reset_counter)) = end.overflow_min {
                if reset_counter == 0 {
                    IntervalLinkResult::EndWithOverflow(val)
                } else {
                    IntervalLinkResult::EndWithoutOverflow(end.no_overflow_min)
                }
            } else {
                IntervalLinkResult::EndWithoutOverflow(end.no_overflow_min)
            }
        }
        (None, None) => IntervalLinkResult::Empty,
    }
}
