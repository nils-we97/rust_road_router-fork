use rust_road_router::datastr::graph::time_dependent::Timestamp;

/// internal data structure to store the separate interval buckets
pub struct MultiLevelBucketTree {
    pub root: BucketTreeEntry, // root is lowerbound profile that spans the whole day
}

impl MultiLevelBucketTree {
    pub fn new(root: BucketTreeEntry) -> Self {
        Self { root }
    }

    // additional global operations here, e.g. tree depth
}

#[derive(Clone, Debug)]
pub struct BucketTreeEntry {
    pub interval_start: Timestamp,      // inclusive interval start
    pub interval_end: Timestamp,        // exclusive interval end
    pub metric_index: usize,            // points to index in metric collection
    pub children: Vec<BucketTreeEntry>, // invariant: each children has the same interval length
}

impl BucketTreeEntry {
    pub fn new(interval_start: Timestamp, interval_end: Timestamp, metric_index: usize, children: Vec<BucketTreeEntry>) -> Self {
        Self {
            interval_start,
            interval_end,
            metric_index,
            children,
        }
    }

    pub fn find_interval(&self, start: Timestamp, end: Timestamp) -> Option<&BucketTreeEntry> {
        if start < self.interval_start || end >= self.interval_end {
            // the current node does not span the whole interval => nothing to return here
            None
        } else {
            // check if it is possible to tighten the interval
            let mut current_entry = Some(self);
            while let Some(entry) = current_entry {
                current_entry = entry
                    .children
                    .iter()
                    .filter(|entry| entry.interval_start <= start && entry.interval_end < end)
                    .next();
            }

            // fallback to the current entry
            Some(current_entry.unwrap_or(&self))
        }
    }
}
