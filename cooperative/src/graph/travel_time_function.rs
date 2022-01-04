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
}

#[inline(always)]
fn is_fifo_violated(x1: Timestamp, x2: Timestamp, y1: Weight, y2: Weight) -> bool {
    // fifo is violated if slope is smaller than -1 => use additional check for overflow-safe code
    y2 < y1 && (y1 - y2) > (x2 - x1)
}

/*fn find_intersection(orig_point: (Timestamp, Weight), latest: (Timestamp, Weight), next: (Timestamp, Weight)) -> (Timestamp, Weight) {
    /*if latest_adjustment != 0 {
        let adjusted_intersection = find_intersection(
            (timestamps[idx], weights[idx]),
            (overflow + timestamps[latest_adjustment], latest_original_weight),
            (timestamps[compare_idx], weights[compare_idx]),
        );
        debug_assert!(!is_fifo_violated(
            timestamps[idx],
            adjusted_intersection.0,
            weights[idx],
            adjusted_intersection.1
        ));

        timestamps[latest_adjustment] = adjusted_intersection.0;
        weights[latest_adjustment] = adjusted_intersection.1;
    }*/

    // convert everything to float before starting, otherwise rounding errors will lead to wrong results
    let x1 = convert_timestamp_u32_to_f64(orig_point.0);
    let y1 = convert_timestamp_u32_to_f64(orig_point.1);

    let x2 = convert_timestamp_u32_to_f64(latest.0);
    let y2 = convert_timestamp_u32_to_f64(latest.1);

    let tx = convert_timestamp_u32_to_f64(next.0);
    let ty = convert_timestamp_u32_to_f64(next.1);

    // use shorthand formula
    let neg_line = LineInterval::line(Line::new((x1, y1).into(), (x1 + 1.0, y1 - 1.0).into()));
    let point_line = LineInterval::ray(Line::new((x2, y2).into(), (tx, ty).into()));
    let intersection = neg_line.relate(&point_line).unique_intersection().unwrap().0;

    (
        convert_timestamp_f64_to_u32(intersection.x.ceil()),
        convert_timestamp_f64_to_u32(intersection.y.ceil()),
    )
}*/

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
