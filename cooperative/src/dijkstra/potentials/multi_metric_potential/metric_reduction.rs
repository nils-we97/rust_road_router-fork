use std::cmp::{min, Ordering};
use std::time::Instant;

use rayon::prelude::*;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;

use rust_road_router::datastr::index_heap::{IndexdMinHeap, Indexing};
use rust_road_router::datastr::rank_select_map::{BitVec, RankSelectMap};

#[derive(Clone, Debug)]
pub struct MetricEntry {
    pub start: Timestamp,
    pub end: Timestamp,
    pub metric_id: usize,
}

impl MetricEntry {
    pub fn new(start: Timestamp, end: Timestamp, metric_id: usize) -> Self {
        Self { start, end, metric_id }
    }
}

#[derive(Debug, Clone, Eq, PartialEq)]
struct MetricItem {
    id: usize,
    difference: u64,
}

impl MetricItem {
    pub fn new(id: usize, difference: u64) -> Self {
        Self { id, difference }
    }
}

impl PartialOrd for MetricItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if self.difference < other.difference {
            Some(Ordering::Less)
        } else if self.difference == other.difference {
            Some(Ordering::Equal)
        } else {
            Some(Ordering::Greater)
        }
    }

    fn lt(&self, other: &Self) -> bool {
        self.difference < other.difference
    }

    fn le(&self, other: &Self) -> bool {
        self.difference <= other.difference
    }

    fn gt(&self, other: &Self) -> bool {
        self.difference > other.difference
    }

    fn ge(&self, other: &Self) -> bool {
        self.difference >= other.difference
    }
}

impl Ord for MetricItem {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }

    fn max(self, other: Self) -> Self
    where
        Self: Sized,
    {
        if self.difference <= other.difference {
            other
        } else {
            self
        }
    }

    fn min(self, other: Self) -> Self
    where
        Self: Sized,
    {
        if self.difference >= other.difference {
            other
        } else {
            self
        }
    }

    fn clamp(self, _min: Self, _max: Self) -> Self
    where
        Self: Sized,
    {
        self
    }
}

impl Indexing for MetricItem {
    fn as_index(&self) -> usize {
        self.id
    }
}

/// reduce the given metrics until only `num_metrics` remain, use a square sum approach
pub fn reduce_metrics(data: &mut Vec<Vec<Weight>>, entries: &mut Vec<MetricEntry>, num_allowed_metrics: usize) -> usize {
    // insert all entry pairs into the priority queue
    let num_metrics = data[0].len();
    let mut queue = IndexdMinHeap::new(num_metrics * num_metrics);

    let start = Instant::now();
    let queue_items: Vec<Vec<MetricItem>> = (0..entries.len() - 1)
        .into_par_iter()
        .map(|i: usize| {
            ((i + 1)..entries.len())
                .into_iter()
                .map(|j| {
                    // avoid another parallel construct - calculation is already parallelized
                    let diff = evaluate_metric_differences(data, entries[i].metric_id, entries[j].metric_id, false);
                    let id = entries[i].metric_id * num_metrics + entries[j].metric_id;
                    MetricItem::new(id, diff)
                })
                .collect::<Vec<MetricItem>>()
        })
        .collect::<Vec<Vec<MetricItem>>>();
    let time = start.elapsed();
    println!("Initialized all metric comparisons in {}s", time.as_secs_f64());

    // remember all deleted metric ids
    let highest_metric_id = entries.iter().max_by_key(|m| m.metric_id).map(|m| m.metric_id).unwrap();
    let mut metric_deactivated = vec![false; highest_metric_id + 1];
    let mut num_active_metrics = entries.len();

    // reduce pairs with a difference of 0 right away, only insert non-zero values in the queue
    // exploit the order (a, b) where a < b: if b is merged right away, it will be known before b's entries are inspected
    queue_items.iter().for_each(|items| {
        debug_assert!(!items.is_empty());
        let first_id = items[0].id / num_metrics;
        debug_assert!(items.iter().all(|entry| entry.id / num_metrics == first_id));

        if !metric_deactivated[first_id] {
            items.iter().for_each(|item| {
                let first_id = item.id / num_metrics;
                let second_id = item.id % num_metrics;

                if !metric_deactivated[first_id] && !metric_deactivated[second_id] {
                    if item.difference == 0 {
                        // merge metric
                        println!("Pre-Merge of metric {} and {}", first_id, second_id);
                        metric_deactivated[second_id] = true;
                        num_active_metrics -= 1;

                        entries.iter_mut().for_each(|entry| {
                            if entry.metric_id == second_id {
                                entry.metric_id = first_id;
                            }
                        });
                    } else {
                        // push entry into queue
                        queue.push(item.clone());
                    }
                }
            });
        }
    });

    println!("Pushed all items into priority queue!");

    while num_active_metrics > num_allowed_metrics {
        if let Some(next) = queue.pop() {
            // extract ids of first and second metric
            let first_id = next.id / num_metrics;
            let second_id = next.id % num_metrics;

            // ignore the entry if one of the corresponding ids is already deactivated!
            if metric_deactivated[first_id] || metric_deactivated[second_id] || first_id == second_id {
                continue;
            }

            println!("Merged metrics {} - {} (diff: {})", first_id, second_id, next.difference);

            // merge the metrics together: after merging, `next.first_id` provides a lowerbound for both time ranges
            merge_metrics(data, first_id, second_id);

            // change metric id in entry structure, mark the second metric as deactivated
            metric_deactivated[second_id] = true;
            num_active_metrics -= 1;

            // collect all entries which have `next.second_id` as their metric id,
            // modify those pointers
            entries.iter_mut().for_each(|entry| {
                if entry.metric_id == second_id {
                    entry.metric_id = first_id;
                }
            });

            // re-calculate the distance from `next.first_id` to all other metrics who haven't been deactivated yet!
            let mut temp_metrics_solved = vec![false; num_metrics];
            entries.iter().for_each(|entry| {
                if first_id != entry.metric_id && !metric_deactivated[entry.metric_id] && !temp_metrics_solved[entry.metric_id] {
                    temp_metrics_solved[entry.metric_id] = true;

                    let diff = evaluate_metric_differences(data, entry.metric_id, first_id, true);
                    let id = if first_id < entry.metric_id {
                        first_id * num_metrics + entry.metric_id
                    } else {
                        entry.metric_id * num_metrics + first_id
                    };
                    let item = MetricItem::new(id, diff);

                    debug_assert!(queue.contains_index(id));
                    queue.update_key(item);
                }
            });
        } else {
            println!("Invalid state reached!");
            break;
        }
    }
    println!("Successfully merged metrics. Rebuilding data structures..");

    // re-build edge metrics, remove deactivated metric values
    // lower and upper bound must not be deactivated!
    debug_assert!(!metric_deactivated[0] && !metric_deactivated[1]);

    data.par_iter_mut().for_each(|edge_metrics| {
        *edge_metrics = (0..=highest_metric_id)
            .into_iter()
            .filter_map(|metric_id| if metric_deactivated[metric_id] { None } else { Some(edge_metrics[metric_id]) })
            .collect::<Vec<Weight>>();
    });

    // also adjust the metric indices in the entry structure
    let mut bit_vec = BitVec::new(metric_deactivated.len());
    for i in 0..metric_deactivated.len() {
        if !metric_deactivated[i] {
            bit_vec.set(i);
        }
    }
    let rank_select_map = RankSelectMap::new(bit_vec);

    entries.iter_mut().for_each(|entry| {
        debug_assert!(!metric_deactivated[entry.metric_id]);
        entry.metric_id = rank_select_map.at(entry.metric_id);
    });

    // number of remaining metrics: all those who haven't been deactivated
    metric_deactivated.iter().filter(|&&v| !v).count()
}

/// merge metric `metric_idx` with `other_metric_idx`, i.e. take the minimum and store it on `metric_idx`
fn merge_metrics(metrics: &mut Vec<Vec<Weight>>, metric_idx: usize, other_metric_idx: usize) {
    metrics
        .iter_mut()
        .for_each(|edge_metrics| edge_metrics[metric_idx] = min(edge_metrics[metric_idx], edge_metrics[other_metric_idx]));
}

fn evaluate_metric_differences(metrics: &Vec<Vec<Weight>>, metric1: usize, metric2: usize, use_parallel_iter: bool) -> u64 {
    if use_parallel_iter {
        metrics
            .par_iter()
            .map(|edge_metrics: &Vec<u32>| {
                let abs_diff = if edge_metrics[metric1] < edge_metrics[metric2] {
                    edge_metrics[metric2] - edge_metrics[metric1]
                } else {
                    edge_metrics[metric1] - edge_metrics[metric2]
                };

                (abs_diff as u64 / 500).pow(2)
            })
            .sum()
    } else {
        metrics
            .iter()
            .map(|edge_metrics| {
                let abs_diff = if edge_metrics[metric1] < edge_metrics[metric2] {
                    edge_metrics[metric2] - edge_metrics[metric1]
                } else {
                    edge_metrics[metric1] - edge_metrics[metric2]
                };

                (abs_diff as u64 / 500).pow(2)
            })
            .sum()
    }
}
