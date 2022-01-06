use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::graph::{Capacity, Velocity, MAX_BUCKETS};
use crate::util::profile_search::find_profile_index;

#[derive(Debug, Clone)]
pub enum CapacityBuckets {
    Unused,
    Used(Vec<(Timestamp, Capacity)>),
}

impl CapacityBuckets {
    pub fn is_used(&self) -> bool {
        if let CapacityBuckets::Used(_) = self {
            true
        } else {
            false
        }
    }

    pub fn inner(&mut self) -> &mut Vec<(Timestamp, Capacity)> {
        if let CapacityBuckets::Used(ret) = self {
            ret
        } else {
            panic!("Wrong type: cannot retrieve inner value of unused capacity bucket!")
        }
    }

    pub fn get(&self, ts: Timestamp) -> Option<Capacity> {
        if let CapacityBuckets::Used(buckets) = self {
            let idx = self.get_index(ts);
            if idx.is_ok() {
                Some(buckets[idx.unwrap()].1)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn get_index(&self, ts: Timestamp) -> Result<usize, usize> {
        debug_assert!(ts <= MAX_BUCKETS, "Timestamp must be in range [0, {}]", MAX_BUCKETS);
        if let CapacityBuckets::Used(buckets) = self {
            if buckets.len() <= 30 {
                // if there are less than 30 buckets, use linear search to find the index / position where to update
                // use custom implementation without sentinels here -> explicit bound checking
                let mut idx = ((ts * (buckets.len() as u32)) / MAX_BUCKETS) as usize;

                if buckets[idx].0 < ts {
                    // increase idx until current timestamp is equal to or greater than ts
                    while idx < buckets.len() && buckets[idx].0 < ts {
                        idx += 1;
                    }

                    return if idx < buckets.len() && buckets[idx].0 == ts {
                        Ok(idx)
                    } else {
                        Err(idx) // next timestamp would be greater, so we have to insert exactly here
                    };
                } else {
                    // decrease idx until current timestamp is less or equal to ts
                    while buckets[idx].0 > ts {
                        if idx == 0 {
                            return Err(0);
                        }
                        idx -= 1;
                    }

                    return if buckets[idx].0 == ts {
                        Ok(idx)
                    } else {
                        Err(idx + 1) // careful: we have decreased one step too far
                    };
                }
            } else {
                // if there are more than 30 buckets, use binary search (builtin)
                return buckets.binary_search_by_key(&ts, |&(bucket_ts, _)| bucket_ts);
            }
        } else {
            return Err(0); // no buckets exist
        };
    }

    /// increment the capacity at `ts` by one
    /// returns true if the bucket was not used before
    pub fn increment(&mut self, ts: Timestamp) -> (bool, usize) {
        let idx = self.get_index(ts);

        if self.is_used() {
            let buckets = self.inner();
            if idx.is_ok() {
                buckets[idx.unwrap()].1 += 1;
                (false, idx.unwrap())
            } else {
                buckets.insert(idx.unwrap_err(), (ts, 1));
                (true, idx.unwrap_err())
            }
        } else {
            *self = CapacityBuckets::Used(vec![(ts, 1)]);
            (true, 0)
        }
    }
}

#[derive(Debug, Clone)]
pub enum SpeedBuckets {
    One(Velocity),
    Many(Vec<(Timestamp, Velocity)>),
}

impl SpeedBuckets {
    pub fn many(&mut self) -> &mut Vec<(Timestamp, Velocity)> {
        if let SpeedBuckets::Many(ret) = self {
            ret
        } else {
            panic!("Wrong type: Expected `Many`, found `One`")
        }
    }

    pub fn one(&mut self) -> &mut Velocity {
        if let SpeedBuckets::One(ret) = self {
            ret
        } else {
            panic!("Wrong type: Expected `One`, found `Many`")
        }
    }

    pub fn get(&self, ts: Timestamp) -> Velocity {
        match self {
            SpeedBuckets::One(velocity) => *velocity,
            SpeedBuckets::Many(buckets) => {
                let idx = self.get_index(ts);
                if idx.is_ok() {
                    buckets[idx.unwrap()].1
                } else {
                    buckets[idx.unwrap_err() - 1].1
                }
            }
        }
    }

    pub fn get_index(&self, ts: Timestamp) -> Result<usize, usize> {
        if let SpeedBuckets::Many(buckets) = self {
            find_profile_index(buckets, ts)
        } else {
            Err(0)
        }
    }

    pub fn update(&mut self, ts: Timestamp, velocity: Velocity, next_ts: Timestamp, freeflow_speed: Velocity) {
        let result = self.get_index(ts);
        match self {
            SpeedBuckets::One(_) => {
                // this is a bit nasty: depending on the daytime, several speed buckets have to be created
                if ts == 0 {
                    // case 1: adjustment at midnight bucket => also consider last sentinel element!
                    *self = SpeedBuckets::Many(vec![(0, velocity), (next_ts, freeflow_speed), (MAX_BUCKETS, velocity)]);
                } else if next_ts == 0 {
                    // case 2: next period would be midnight => don't create midnight bucket twice!
                    *self = SpeedBuckets::Many(vec![(0, freeflow_speed), (ts, velocity), (MAX_BUCKETS, freeflow_speed)]);
                } else {
                    // case 3 (standard): edge gets passed nowhere close to midnight
                    *self = SpeedBuckets::Many(vec![
                        (0, freeflow_speed),
                        (ts, velocity),
                        (next_ts, freeflow_speed),
                        (MAX_BUCKETS, freeflow_speed),
                    ]);
                }
            }
            SpeedBuckets::Many(ref mut buckets) => {
                // check whether the respective bucket already exists
                let idx: usize;

                if result.is_ok() {
                    // bucket exists => increase capacity by 1 vehicle and update speed
                    idx = result.unwrap();
                    buckets[idx].1 = velocity;

                    // if the change occurs at midnight, then also update the sentinel element
                    if idx == 0 {
                        let last_idx = buckets.len() - 1;
                        buckets[last_idx].1 = velocity;
                    }
                } else {
                    idx = result.unwrap_err();
                    buckets.insert(idx, (ts, velocity));
                }

                // additionally, it is required to check whether the neighboring bucket already exists
                // if this bucket does not exist already, the TTF algorithm would falsely
                // assume that the speed is the same within the next bucket(s)
                if next_ts != 0 && buckets[idx + 1].0 != next_ts {
                    // only update if the next existing bucket ts != `next_ts`
                    buckets.insert(idx + 1, (next_ts, freeflow_speed));
                }
            }
        }
    }
}
