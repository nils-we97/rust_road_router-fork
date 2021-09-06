use crate::graph::MAX_BUCKETS;
use rust_road_router::datastr::graph::floating_time_dependent::TTFPoint;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use std::cmp::Ordering;
use std::fmt::Debug;

pub trait ProfileIndex<K: Debug> {
    fn ts(&self) -> K;
    fn zero() -> K;
    fn period() -> K;
    fn lin_search_start(val: &K, len: usize) -> usize;
    fn bin_search_ord(&self, val: &K) -> Ordering;
}

impl<T> ProfileIndex<Timestamp> for (Timestamp, T) {
    fn ts(&self) -> Timestamp {
        self.0
    }
    fn zero() -> Timestamp {
        0
    }
    fn period() -> Timestamp {
        MAX_BUCKETS
    }
    fn lin_search_start(val: &Timestamp, len: usize) -> usize {
        (((len as u32) * *val) / MAX_BUCKETS) as usize
    }

    fn bin_search_ord(&self, val: &u32) -> Ordering {
        self.0.cmp(val)
    }
}

impl ProfileIndex<f64> for TTFPoint {
    fn ts(&self) -> f64 {
        self.at.0
    }
    fn zero() -> f64 {
        0.0
    }
    fn period() -> f64 {
        86400.0
    }
    fn lin_search_start(val: &f64, len: usize) -> usize {
        ((*val / 86400.0) * (len as f64)) as usize
    }

    fn bin_search_ord(&self, val: &f64) -> Ordering {
        self.at.0.partial_cmp(val).unwrap_or(Ordering::Equal)
    }
}

/// Searches a sorted array for a given timestamp entry.
/// If the exact timestamp exists, it will be returned. Otherwise, the correct place for insertion will be returned.
///
/// Current implementation: for small arrays (len < 30), linear search seems faster. Otherwise use binary search.
///
/// Careful: it is assumed that the first timestamp of `arr` is zero and the latest equals `MAX_BUCKETS`.
/// This simplified assumption avoids some edges cases and is thus more efficient
pub fn find_profile_index<K: PartialEq + PartialOrd + Debug, T: ProfileIndex<K> + Debug>(arr: &[T], ts: K) -> Result<usize, usize> {
    // treat easy sentinel edge cases first
    if ts == T::zero() {
        Ok(0)
    } else if ts == T::period() {
        Ok(arr.len() - 1)
    } else if arr.len() <= 30 {
        let mut idx = T::lin_search_start(&ts, arr.len());
        debug_assert!(idx < arr.len(), "{:#?}", (arr, ts));

        if arr[idx].ts() == ts {
            return Ok(idx);
        }

        return if arr[idx].ts() < ts {
            // linear search, increase idx until the current timestamp is equal to or greater than ts
            while arr[idx].ts() < ts {
                idx += 1;
            }
            if arr[idx].ts() == ts {
                Ok(idx)
            } else {
                Err(idx) // next timestamp would be greater, so we have to insert exactly here
            }
        } else {
            // linear search, decrease idx until the current timestamp is equal to or less than ts
            while arr[idx].ts() > ts {
                idx -= 1;
            }
            if arr[idx].ts() == ts {
                Ok(idx)
            } else {
                Err(idx + 1)
            }
        };
    } else {
        arr.binary_search_by(|a| a.bin_search_ord(&ts))
    }
}
