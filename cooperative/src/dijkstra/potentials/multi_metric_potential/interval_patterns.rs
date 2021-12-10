use rust_road_router::datastr::graph::time_dependent::Timestamp;

/// balanced pattern: 8 hours, 4 hours, 2 hours, 1 hour
pub fn balanced_interval_pattern() -> Vec<(Timestamp, Timestamp)> {
    let mut a = (0..5)
        .into_iter()
        .map(|i| (ts_from(i * 4, 0), ts_from(i * 4 + 8, 0)))
        .collect::<Vec<(Timestamp, Timestamp)>>();

    let b = (0..11)
        .into_iter()
        .map(|i| (ts_from(i * 2, 0), ts_from(i * 2 + 4, 0)))
        .collect::<Vec<(Timestamp, Timestamp)>>();

    let c = (0..23)
        .into_iter()
        .map(|i| (ts_from(i, 0), ts_from(i + 2, 0)))
        .collect::<Vec<(Timestamp, Timestamp)>>();

    let d = (0..47)
        .into_iter()
        .map(|i| (ts_from(0, i * 30), ts_from(0, (i + 1) * 30)))
        .collect::<Vec<(Timestamp, Timestamp)>>();

    a.extend_from_slice(&b);
    a.extend_from_slice(&c);
    a.extend_from_slice(&d);
    a
}

pub fn rush_hour_pattern() -> Vec<(Timestamp, Timestamp)> {
    vec![
        (ts_from(0, 0), ts_from(6, 0)),
        (ts_from(20, 0), ts_from(24, 0)),
        (ts_from(6, 0), ts_from(10, 0)),
        (ts_from(10, 0), ts_from(15, 0)),
        (ts_from(10, 0), ts_from(10, 12)),
        (ts_from(12, 0), ts_from(14, 0)),
        (ts_from(11, 0), ts_from(13, 0)),
        (ts_from(15, 0), ts_from(20, 0)),
        (ts_from(7, 0), ts_from(9, 0)),
        (ts_from(7, 0), ts_from(8, 0)),
        (ts_from(8, 0), ts_from(9, 0)),
        (ts_from(15, 0), ts_from(17, 0)),
        (ts_from(17, 0), ts_from(19, 0)),
        (ts_from(19, 0), ts_from(21, 0)),
        (ts_from(16, 0), ts_from(17, 0)),
        (ts_from(17, 0), ts_from(18, 0)),
        (ts_from(18, 0), ts_from(19, 0)),
        (ts_from(19, 0), ts_from(20, 0)),
    ]
}

#[inline(always)]
fn ts_from(hour: u32, minute: u32) -> Timestamp {
    hour * 3_600_000 + minute * 60_000
}
