use rust_road_router::datastr::graph::{Weight, INFINITY};
use std::cmp::min;

#[derive(Clone, Debug)]
pub struct IntervalMinimaTree {
    pub values: Vec<Vec<Weight>>,
    pub num_values_per_level: u32,
}

impl IntervalMinimaTree {
    pub fn new(intervals: &Vec<Weight>) -> Self {
        // keep it simple for now: assume that values is a power of 2
        let len = intervals.len() as u32;
        debug_assert_eq!(len & (len - 1), 0, "Length of `values` must be a power of 2!");

        // determine number of levels (log2(intervals.len()))
        let num_levels = 31 - len.leading_zeros();
        let mut values = vec![vec![0; intervals.len()]; num_levels as usize];

        // first level is simple, just take the original values
        values[0] = intervals.clone();

        for level in 1..num_levels {
            let mut forward_direction = true;
            let mut current_step = 0;
            let mut current_step_min = INFINITY;
            let step_width = 2u32.pow(level);

            for current_index in 0..len {
                if forward_direction {
                    // calculate postfix maxima within the current step width
                    let idx = current_index + step_width - 1 - 2 * current_step;

                    current_step_min = min(current_step_min, intervals[idx as usize]);
                    values[level as usize][idx as usize] = current_step_min;
                } else {
                    // calculate prefix minima within the current step width
                    current_step_min = min(current_step_min, intervals[current_index as usize]);
                    values[level as usize][current_index as usize] = current_step_min;
                }

                // update step and direction
                current_step += 1;
                if current_step == step_width {
                    // switch direction
                    current_step = 0;
                    forward_direction = !forward_direction;
                    current_step_min = INFINITY;
                }
            }
        }

        Self {
            values,
            num_values_per_level: len,
        }
    }

    pub fn find_interval_minimum(&self, idx1: usize, idx2: usize) -> Weight {
        if idx1 == idx2 {
            // easy case first: indices are equal, simply take the value on level 0
            self.values[0][idx1]
        } else if idx1 < idx2 {
            // consecutive interval, common case
            // take the highest level where idx1 and idx2 are not processed in the same step/direction
            // this can be done by simply taking the most significant bit difference
            let level = 31 - ((idx1 as u32) ^ (idx2 as u32)).leading_zeros();
            min(self.values[level as usize][idx1], self.values[level as usize][idx2])
        } else {
            // edge case: intervals cross midnight -> recurse (at most one time!)
            // todo check whether recursion is fast enough
            min(
                self.find_interval_minimum(idx1, self.num_values_per_level as usize - 1),
                self.find_interval_minimum(0, idx2),
            )
        }
    }
}

#[test]
fn leading_zeros() {
    let values = vec![88, 56, 30, 98, 15, 65, 75, 56, 34, 52, 77, 41, 62, 74, 76, 80];

    let result = IntervalMinimaTree::new(&values);
    let test = result.find_interval_minimum(10, 5);

    dbg!(&result, test);
}
