use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::graph::{Capacity, Velocity, MAX_BUCKETS};

#[derive(Debug, Clone)]
pub enum CapacityBuckets {
    Unused,
    Used(Vec<(Timestamp, Capacity)>),
}

impl CapacityBuckets {
    pub fn is_used(&self) -> bool {
        match self {
            CapacityBuckets::Unused => false,
            CapacityBuckets::Used(_) => true,
        }
    }

    pub fn inner(&mut self) -> &mut Vec<(Timestamp, Capacity)> {
        match self {
            CapacityBuckets::Unused => unimplemented!(),
            CapacityBuckets::Used(inner) => inner,
        }
    }

    /// increment the capacity at `ts` by one and returns the updated value
    pub fn increment(&mut self, ts: Timestamp) -> Capacity {
        match self {
            CapacityBuckets::Unused => {
                *self = CapacityBuckets::Used(vec![(ts, 1)]);
                1
            }
            CapacityBuckets::Used(inner) => {
                let pos = inner.binary_search_by_key(&ts, |&(bucket_ts, _)| bucket_ts);

                if let Ok(pos) = pos {
                    inner[pos].1 += 1;
                    inner[pos].1
                } else if let Err(pos) = pos {
                    inner.insert(pos, (ts, 1));
                    1
                } else {
                    unimplemented!()
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
pub enum SpeedBuckets {
    Unused,
    Used(Vec<(Timestamp, Velocity)>),
}

impl SpeedBuckets {
    pub fn is_used(&self) -> bool {
        match self {
            SpeedBuckets::Unused => false,
            SpeedBuckets::Used(_) => true,
        }
    }

    pub fn inner(&mut self) -> &mut Vec<(Timestamp, Velocity)> {
        match self {
            SpeedBuckets::Unused => unimplemented!(),
            SpeedBuckets::Used(data) => data,
        }
    }

    pub fn update(&mut self, ts: Timestamp, velocity: Velocity, next_ts: Timestamp, freeflow_speed: Velocity) {
        match self {
            SpeedBuckets::Unused => {
                // initialize new speed bucket container, depending on `ts`
                if ts == 0 {
                    // case 1: adjustment at midnight bucket => also consider last sentinel element!
                    debug_assert!(next_ts != 0 && next_ts != MAX_BUCKETS);
                    *self = SpeedBuckets::Used(vec![(0, velocity), (next_ts, freeflow_speed), (MAX_BUCKETS, velocity)]);
                } else if next_ts == 0 {
                    // case 2: next period would be midnight => don't create midnight bucket twice!
                    *self = SpeedBuckets::Used(vec![(0, freeflow_speed), (ts, velocity), (MAX_BUCKETS, freeflow_speed)]);
                } else {
                    // case 3: regular case
                    *self = SpeedBuckets::Used(vec![
                        (0, freeflow_speed),
                        (ts, velocity),
                        (next_ts, freeflow_speed),
                        (MAX_BUCKETS, freeflow_speed),
                    ]);
                }
            }
            SpeedBuckets::Used(inner) => {
                let pos = inner.binary_search_by_key(&ts, |&(entry_ts, _)| entry_ts);

                if let Ok(pos) = pos {
                    // position already exists -> simply update
                    inner[pos].1 = velocity;

                    // update sentinel if change occurs at midnight
                    if ts == 0 {
                        (*inner.last_mut().unwrap()).1 = velocity;
                    }

                    // check whether a new entry has to be appended
                    if next_ts != 0 && inner[pos + 1].0 != next_ts {
                        // only update if the next existing bucket ts != `next_ts`
                        inner.insert(pos + 1, (next_ts, freeflow_speed));
                    }
                } else if let Err(pos) = pos {
                    debug_assert!(pos > 0);
                    inner.insert(pos, (ts, velocity));

                    // check whether a new entry has to be appended
                    if next_ts != 0 && inner[pos + 1].0 != next_ts {
                        // only update if the next existing bucket ts != `next_ts`
                        inner.insert(pos + 1, (next_ts, freeflow_speed));
                    }
                }
            }
        }
    }
}
