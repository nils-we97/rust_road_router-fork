use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;

use crate::dijkstra::potentials::multi_level_bucket_potential::bucket_tree::{evaluate_metric_differences, merge_metrics};

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
    first_id: usize,
    second_id: usize,
    first_entry_idx: usize,
    second_entry_idx: usize,
    difference: u64,
}

impl MetricItem {
    pub fn new(first_id: usize, second_id: usize, first_entry_idx: usize, second_entry_idx: usize, difference: u64) -> Self {
        Self {
            first_id,
            second_id,
            first_entry_idx,
            second_entry_idx,
            difference,
        }
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

/// reduce the given metrics until only `num_metrics` remain. Use the square sum approach from the `MultiLevelBucket` potential
pub fn reduce_metrics(data: &mut Vec<Vec<Weight>>, entries: &mut Vec<MetricEntry>, num_metrics: usize) -> usize {
    // insert all entry pairs into the priority queue
    let mut queue = BinaryHeap::with_capacity(entries.len() * entries.len());
    for i in 0..entries.len() {
        for j in (i + 1)..entries.len() {
            let diff = evaluate_metric_differences(data, entries[i].metric_id, entries[j].metric_id);
            queue.push(MetricItem::new(entries[i].metric_id, entries[j].metric_id, i, j, diff));
        }
    }

    // remember all deleted metric ids
    let max_metric_id = entries.iter().max_by_key(|e| e.metric_id).map(|e| e.metric_id).unwrap();
    let mut metric_deactivated = vec![false; max_metric_id + 1];
    let mut num_active_metrics = entries.len();

    while num_active_metrics > num_metrics {
        if let Some(next) = queue.pop() {
            // ignore the entry if one of the corresponding ids is already deactivated!
            if metric_deactivated[next.first_id] || metric_deactivated[next.second_id] {
                continue;
            }

            // merge the metrics together: after merging, `next.first_id` provides a lowerbound for both time ranges
            merge_metrics(data, next.first_id, next.second_id);

            // change metric id in entry structure, mark the second metric as deactivated
            entries[next.second_entry_idx].metric_id = next.first_id;
            metric_deactivated[next.second_id] = true;
            num_active_metrics -= 1;

            // collect all queue entries which have to be updated
            // and re-insert them with updated metric differences
            let mut metrics_to_update = Vec::new();

            // there are two cases to consider:
            // a) there is an entry (`next.first_id`, other) where `other` hasn't been deactivated yet
            // -> we have to recalculate the difference between `next.first_id` and `other`
            // b) there is an entry (other, `next.second_id`) where `other` hasn't been deactivated yet
            // -> we have to recalculate the difference between `other` and `next.first_id` (second is merged!)
            for item in &queue {
                if (item.first_id == next.first_id || item.first_id == next.second_id) && !metric_deactivated[item.second_id] {
                    // case 1: entry (first_id, x) no longer valid -> recalculate (first_id, x)
                    metrics_to_update.push((item.second_id, item.second_entry_idx));
                } else if (item.second_id == next.first_id || item.second_id == next.second_id) && !metric_deactivated[item.first_id] {
                    // case 2: (
                    metrics_to_update.push((item.first_id, item.first_entry_idx));
                }
            }

            for (metric_id, entry_id) in metrics_to_update {
                debug_assert_ne!(metric_id, next.first_id, "metric ids must not be identical!");
                let diff = evaluate_metric_differences(data, metric_id, next.first_id);

                // keep smaller metric ids
                if metric_id < next.first_id {
                    queue.push(MetricItem::new(metric_id, next.first_id, entry_id, next.first_entry_idx, diff));
                } else {
                    queue.push(MetricItem::new(next.first_id, metric_id, next.first_entry_idx, entry_id, diff));
                }
            }
        } else {
            println!("Invalid state reached!");
            break;
        }
    }

    // re-build edge metrics, remove deactivated metric values
    // lower and upper bound must not be deactivated!
    debug_assert!(!metric_deactivated[0] && !metric_deactivated[1]);

    data.iter_mut().for_each(|edge_metrics| {
        *edge_metrics = edge_metrics
            .iter()
            .enumerate()
            .filter(|&(metric_id, _)| !metric_deactivated[metric_id])
            .map(|(_, &val)| val)
            .collect::<Vec<Weight>>();
    });

    // number of remaining metrics: all those who haven't been deactivated
    metric_deactivated.iter().filter(|&&v| !v).count()
}
