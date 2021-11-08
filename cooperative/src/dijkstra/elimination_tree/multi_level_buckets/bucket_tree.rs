use crate::graph::MAX_BUCKETS;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;
use std::cmp::min;
use std::ops::Range;

#[derive(Clone, Debug)]
pub struct MultiLevelBucketTree {
    pub elements: Vec<MultiLevelBucketTreeEntry>,
    pub children: Vec<usize>,
    pub parents: Vec<usize>,
    pub interval_lengths: Vec<Timestamp>,
    pub num_entries_per_level: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct MultiLevelBucketTreeEntry {
    pub interval_start: Timestamp,
    pub interval_end: Timestamp,
    pub metric_id: usize,
}

impl MultiLevelBucketTreeEntry {
    pub fn new(interval_start: Timestamp, interval_end: Timestamp, metric_id: usize) -> Self {
        Self {
            interval_start,
            interval_end,
            metric_id,
        }
    }
}

impl MultiLevelBucketTree {
    pub fn new(interval_lengths: &Vec<Timestamp>) -> Self {
        let mut elements = Vec::new();
        let mut children = Vec::new();
        let mut parents = Vec::new();

        // 1. add a separate level for the root
        let mut extended_interval_lengths = vec![MAX_BUCKETS];
        extended_interval_lengths.extend_from_slice(interval_lengths);

        // 2. determine how many entries exist on each level
        let num_entries_per_level = extended_interval_lengths
            .iter()
            .map(|&len| (MAX_BUCKETS / len) as usize)
            .collect::<Vec<usize>>();

        // 3. use the `num_entries_per_level` vec to determine how many children a node has on each level
        let mut num_children_per_level = (0..extended_interval_lengths.len() - 1)
            .into_iter()
            .map(|idx| num_entries_per_level[idx + 1] / num_entries_per_level[idx])
            .collect::<Vec<usize>>();
        num_children_per_level.push(0); // last level consists of leaves only

        // 4. initialize dummy elements for children and parents
        children.push(1);
        parents.push(0);

        // 5. step over each level and add the remaining data
        let mut current_level_start_idx = 0;
        for level in 0..extended_interval_lengths.len() {
            for i in 0..num_entries_per_level[level] {
                // add the current element
                // special case treatment for root level because the global upperbounds are at position 1!
                elements.push(MultiLevelBucketTreeEntry::new(
                    i as u32 * extended_interval_lengths[level],
                    (i + 1) as u32 * extended_interval_lengths[level],
                    if level == 0 { 0 } else { current_level_start_idx + i + 1 },
                ));

                // adjust the children pointers, special case treatment for leaves
                if num_children_per_level[level] > 0 {
                    children.push(*children.last().unwrap() + num_children_per_level[level]);
                } else {
                    children.push(*children.last().unwrap());
                }

                // adjust the parent pointers, special case treatment for root node
                if level > 0 {
                    let parent_level_start_index = current_level_start_idx - num_entries_per_level[level - 1];
                    parents.push(parent_level_start_index + (i / num_children_per_level[level - 1]));
                }
            }
            current_level_start_idx += num_entries_per_level[level];
        }

        Self {
            elements,
            children,
            parents,
            num_entries_per_level,
            interval_lengths: extended_interval_lengths,
        }
    }

    pub fn reduce(&mut self, square_sum_threshold: u64, metrics: &mut Vec<Vec<Weight>>) {
        // initialize additional data containers
        let mut element_remaining = vec![true; self.elements.len()];
        let mut prev_level_first_idx = self.elements.len();
        let mut remaining_on_level = self.num_entries_per_level.clone();
        let mut remaining_metrics = vec![true; metrics[0].len()];

        // reduce bottom up, start with the leaves and ignore the root level
        for level in (1..self.num_entries_per_level.len()).into_iter().rev() {
            let current_level_first_index = prev_level_first_idx - self.num_entries_per_level[level];

            let mut compare_base_idx = current_level_first_index;
            let mut compare_next_idx = current_level_first_index + 1;

            // continue to compare elements as long as the right pointer does not overflow
            while compare_next_idx < prev_level_first_idx {
                // only compare the elements if they share the same parent node and have no (remaining) children!
                let num_children_base = self.children_range(compare_base_idx).filter(|&child| element_remaining[child]).count();
                let num_children_next = self.children_range(compare_next_idx).filter(|&child| element_remaining[child]).count();

                if self.parents[compare_base_idx] == self.parents[compare_next_idx] && num_children_base == 0 && num_children_next == 0 {
                    // retrieve the correct metric ids and evaluate their differences
                    let base_metric_idx = self.elements[compare_base_idx].metric_id;
                    let compare_metric_idx = self.elements[compare_next_idx].metric_id;
                    let diff = evaluate_metric_differences(metrics, base_metric_idx, compare_metric_idx);
                    /*println!(
                        "Diff between metric {} and {}: {} ({} per edge)",
                        base_metric_idx,
                        compare_metric_idx,
                        diff,
                        diff / metrics.len() as u64
                    );*/

                    if diff < square_sum_threshold {
                        // extend interval of base idx element!
                        self.elements[compare_base_idx].interval_end = self.elements[compare_next_idx].interval_end;

                        // merge the metrics, take the minimum value of each edge
                        merge_metrics(metrics, base_metric_idx, compare_metric_idx);

                        // deactivate the second metric and only move the next pointer forward
                        element_remaining[compare_next_idx] = false;
                        remaining_metrics[compare_metric_idx] = false;
                        compare_next_idx += 1;
                        remaining_on_level[level] -= 1;
                    } else {
                        // move both pointers forward, current base might skip already deactivated entries here!
                        compare_base_idx = compare_next_idx;
                        compare_next_idx += 1;
                    }
                } else {
                    // move both pointers forward, current base might skip already deactivated entries here!
                    compare_base_idx = compare_next_idx;
                    compare_next_idx += 1;
                }
            }

            // additional adjustment: deactivate a leaf if all its siblings are deactivated too
            let first_parent_idx = current_level_first_index - self.num_entries_per_level[level - 1];

            for parent_idx in first_parent_idx..current_level_first_index {
                let children = self.children_range(parent_idx);
                let num_surviving_children = children.clone().filter(|&child| element_remaining[child]).count();
                if num_surviving_children == 1 {
                    children.for_each(|child| {
                        element_remaining[child] = false;
                        remaining_metrics[self.elements[child].metric_id] = false;
                    });
                    remaining_on_level[level] -= 1;
                }
            }

            // adjust index barrier
            prev_level_first_idx = current_level_first_index;
        }

        // build new tree, adjust parent and children structure as well

        // build prefix sum over remaining metrics
        let mut remaining_metrics_prefix_sum = vec![0];
        remaining_metrics.iter().for_each(|&val| {
            let current = if val { 1 } else { 0 };
            remaining_metrics_prefix_sum.push(*remaining_metrics_prefix_sum.last().unwrap() + current);
        });

        // old elements are still in the correct order, -> collect the surviving ones
        let new_elements = self
            .elements
            .iter()
            .enumerate()
            .filter(|&(idx, _)| element_remaining[idx])
            .map(|(_, entry)| {
                let mut clone = entry.clone();
                clone.metric_id = remaining_metrics_prefix_sum[clone.metric_id];
                clone
            })
            .collect::<Vec<MultiLevelBucketTreeEntry>>();

        // in order to determine the new children array, we can simply iterate over the old elements and collect the number of remaining children
        let mut new_children = vec![1];
        (0..self.elements.len()).into_iter().filter(|&idx| element_remaining[idx]).for_each(|idx| {
            let num_remaining_children = self.children_range(idx).into_iter().filter(|&child| element_remaining[child]).count();
            new_children.push(*new_children.last().unwrap() + num_remaining_children);
        });

        // new parent ids can be retrieved with prefix sums and reverse indexing
        let mut remaining_elements_prefix_sum = vec![0];
        (0..self.elements.len()).into_iter().for_each(|idx| {
            let current = if element_remaining[idx] { 1 } else { 0 };
            remaining_elements_prefix_sum.push(*remaining_elements_prefix_sum.last().unwrap() + current);
        });

        let new_parents = (0..self.elements.len())
            .into_iter()
            .filter(|&idx| element_remaining[idx])
            .map(|idx| remaining_elements_prefix_sum[self.parents[idx]])
            .collect::<Vec<usize>>();

        // update structures
        self.elements = new_elements;
        self.children = new_children;
        self.parents = new_parents;
        self.num_entries_per_level = remaining_on_level;

        // only keep updated metrics
        metrics.iter_mut().for_each(|edge_metrics| {
            *edge_metrics = edge_metrics
                .iter()
                .enumerate()
                .filter(|&(metric_idx, _)| remaining_metrics[metric_idx])
                .map(|(_, &val)| val)
                .collect::<Vec<Weight>>()
        });
    }

    pub fn children_range(&self, node: usize) -> Range<usize> {
        self.children[node]..self.children[node + 1]
    }

    pub fn find_interval(&self, start_ts: Timestamp, end_ts: Timestamp) -> (usize, usize) {
        if start_ts > end_ts {
            // edge case: interval crosses midnight -> always return the lowerbound value here
            (0, 0)
        } else {
            let mut current_idx = Some(0);
            let mut ret = (0, 0);

            // iterative approach: try to descend to a suitable child node
            while let Some(idx) = current_idx {
                ret = (idx, self.elements[idx].metric_id);

                current_idx = self
                    .children_range(idx)
                    .into_iter()
                    .filter(|&child| self.elements[child].interval_start <= start_ts && self.elements[child].interval_end > end_ts)
                    .next();
            }

            ret
        }
    }
}

// helper functions
fn evaluate_metric_differences(metrics: &Vec<Vec<Weight>>, metric1: usize, metric2: usize) -> u64 {
    let mut square_sum = 0;

    metrics.iter().for_each(|edge_metrics| {
        let abs_diff = if edge_metrics[metric1] < edge_metrics[metric2] {
            edge_metrics[metric2] - edge_metrics[metric1]
        } else {
            edge_metrics[metric1] - edge_metrics[metric2]
        } as u64;
        square_sum += abs_diff.pow(2);
    });

    square_sum
}

/// merge metric `metric_idx` with `other_metric_idx`, i.e. take the minimum and store it on `metric_idx`
fn merge_metrics(metrics: &mut Vec<Vec<Weight>>, metric_idx: usize, other_metric_idx: usize) {
    metrics
        .iter_mut()
        .for_each(|edge_metrics| edge_metrics[metric_idx] = min(edge_metrics[metric_idx], edge_metrics[other_metric_idx]));
}
