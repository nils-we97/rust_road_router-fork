use std::cmp::Ordering;

use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;

use crate::dijkstra::potentials::multi_level_bucket_potential::bucket_tree::{evaluate_metric_differences, merge_metrics};
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

/// reduce the given metrics until only `num_metrics` remain. Use the square sum approach from the `MultiLevelBucket` potential
pub fn reduce_metrics(data: &mut Vec<Vec<Weight>>, entries: &mut Vec<MetricEntry>, num_allowed_metrics: usize) -> usize {
    // insert all entry pairs into the priority queue
    let num_metrics = data[0].len();
    let mut queue = IndexdMinHeap::new(num_metrics * num_metrics);

    for i in 0..entries.len() {
        for j in (i + 1)..entries.len() {
            let diff = evaluate_metric_differences(data, entries[i].metric_id, entries[j].metric_id);
            let id = entries[i].metric_id * num_metrics + entries[j].metric_id;
            queue.push(MetricItem::new(id, diff));
        }
    }
    println!("Initialized all metric comparisons!");

    // remember all deleted metric ids
    let mut metric_deactivated = vec![false; data[0].len()];
    let mut num_active_metrics = entries.len();

    while num_active_metrics > num_allowed_metrics {
        if let Some(next) = queue.pop() {
            // extract ids of first and second metric
            let first_id = next.id / num_metrics;
            let second_id = next.id % num_metrics;

            // ignore the entry if one of the corresponding ids is already deactivated!
            if metric_deactivated[first_id] || metric_deactivated[second_id] || first_id == second_id {
                continue;
            }

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
            // start with lowerbound distance
            if first_id != 0 {
                let diff = evaluate_metric_differences(data, 0, first_id);
                let id = first_id;
                let item = MetricItem::new(id, diff);

                if queue.contains_index(id) {
                    queue.update_key(item);
                } else {
                    queue.push(item);
                }
            }
            // skip upper bound at position 1, continue with index 2
            for i in 2..first_id {
                if !metric_deactivated[i] {
                    let diff = evaluate_metric_differences(data, i, first_id);
                    let id = i * num_metrics + first_id;
                    let item = MetricItem::new(id, diff);

                    if queue.contains_index(id) {
                        queue.update_key(item);
                    } else {
                        queue.push(item);
                    }
                }
            }
            for i in (first_id + 1)..metric_deactivated.len() {
                if !metric_deactivated[i] {
                    let diff = evaluate_metric_differences(data, first_id, i);
                    let id = first_id * num_metrics + i;
                    let item = MetricItem::new(id, diff);

                    if queue.contains_index(id) {
                        queue.update_key(item);
                    } else {
                        queue.push(item);
                    }
                }
            }
        } else {
            println!("Invalid state reached!");
            break;
        }
    }
    println!("Successfully merged metrics. Rebuilding data structures..");

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
