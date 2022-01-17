use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;

/// re-build an entire travel time function
pub fn build_ttf(timestamps: &Vec<Timestamp>, weights: &mut Vec<Weight>) {
    let num_breakpoints = timestamps.len() - 1; // ignore sentinel element

    // 1. iterate overall neighbors, fix broken fifo property wherever needed
    for idx in 0..num_breakpoints {
        if is_fifo_violated(timestamps[idx], timestamps[idx + 1], weights[idx], weights[idx + 1]) {
            // move next point upward
            weights[idx + 1] = weights[idx] - (timestamps[idx + 1] - timestamps[idx]);
        }
    }

    // 2. update period boundaries
    weights[0] = *weights.last().unwrap();

    // 3. fix remaining violations beyond midnight
    // it suffices to continue until fifo is restored -> rest is already covered by first run!
    let mut current_idx = 0;
    while is_fifo_violated(
        timestamps[current_idx],
        timestamps[current_idx + 1],
        weights[current_idx],
        weights[current_idx + 1],
    ) {
        weights[current_idx + 1] = weights[current_idx] - (timestamps[current_idx + 1] - timestamps[current_idx]);
        current_idx += 1;
    }

    // 4. update period boundaries again to ensure consistency
    // (and rounding errors induced by transformation capacity -> travel time!)
    *weights.last_mut().unwrap() = weights[0];

    debug_assert!(timestamps
        .windows(2)
        .zip(weights.windows(2))
        .all(|(dep, tt)| !is_fifo_violated(dep[0], dep[1], tt[0], tt[1])));
}

/// updates the travel time function on a single entry
pub fn update_ttf(timestamps: &Vec<Timestamp>, weights: &mut Vec<Weight>, update_pos: usize, period: Timestamp) {
    let num_breakpoints = timestamps.len() - 1;

    let mut current_idx = update_pos;
    let mut next_idx = (update_pos + 1) % num_breakpoints;
    let mut next_ts = timestamps[next_idx] + period * (next_idx == 0) as u32;

    while is_fifo_violated(timestamps[current_idx], next_ts, weights[current_idx], weights[next_idx]) {
        if current_idx < next_idx {
            weights[next_idx] = weights[current_idx] - (next_ts - timestamps[current_idx]);
        } else {
            debug_assert_eq!(next_idx, 0);
            weights[next_idx] = weights[current_idx] - (period - timestamps[current_idx]);
            *weights.last_mut().unwrap() = weights[0];
        }

        current_idx = (current_idx + 1) % num_breakpoints;
        next_idx = (current_idx + 1) % num_breakpoints;
        next_ts = timestamps[next_idx] + period * (next_idx == 0) as u32;
    }

    // keep timestamp boundaries consistent
    weights[0] = *weights.last().unwrap();

    debug_assert!(timestamps
        .windows(2)
        .zip(weights.windows(2))
        .all(|(dep, tt)| !is_fifo_violated(dep[0], dep[1], tt[0], tt[1])));
}

#[inline(always)]
fn is_fifo_violated(x1: Timestamp, x2: Timestamp, y1: Weight, y2: Weight) -> bool {
    // fifo is violated if slope is smaller than -1 => use additional check for overflow-safe code
    debug_assert!(x1 < x2);
    y2 < y1 && (y1 - y2) > (x2 - x1)
}

#[test]
fn test_build_ttf() {
    let timestamps = vec![0, 2, 4, 6, 8, 10];
    let mut weights = vec![4, 6, 8, 3, 2, 4];

    println!("Timestamps: {:?}", timestamps);

    build_ttf(&timestamps, &mut weights);
    println!("Weights after re-building: {:?}", weights);

    weights = vec![4, 11, 8, 6, 4, 4];

    update_ttf(&timestamps, &mut weights, 1, 10);
    println!("Weights after update: {:?}", weights);
}
