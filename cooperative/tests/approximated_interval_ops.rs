use conversion::speed_profile_to_tt_profile;
use cooperative::dijkstra::potentials::corridor_interval_potential::ops::{ApproximatedIntervalLabel, IntervalLabelEntry, TDCorridorIntervalPotentialOps};
use cooperative::graph::MAX_BUCKETS;
use rust_road_router::datastr::graph::{EdgeIdT, Reversed};
use rust_road_router::report::measure;

#[test]
fn approximated_interval_link() {
    let hour = MAX_BUCKETS / 24;

    let edge_profiles = vec![
        speed_profile_to_tt_profile(&vec![(0, 60), (12 * hour, 40), (MAX_BUCKETS, 60)], 1000),
        speed_profile_to_tt_profile(&vec![(0, 30), (8 * hour, 20), (16 * hour, 10), (MAX_BUCKETS, 30)], 500),
    ];

    let departures = edge_profiles
        .iter()
        .map(|val| val.iter().map(|&(a, _)| a).collect::<Vec<u32>>())
        .collect::<Vec<Vec<u32>>>();

    let travel_times = edge_profiles
        .iter()
        .map(|val| val.iter().map(|&(_, b)| b).collect::<Vec<u32>>())
        .collect::<Vec<Vec<u32>>>();

    let ops = TDCorridorIntervalPotentialOps::new(0, 1000, 6, &departures, &travel_times);
    dbg!(&ops);

    let label = ApproximatedIntervalLabel::new(
        Some(hour * 12),
        vec![IntervalLabelEntry::new(15000, None), IntervalLabelEntry::new(30000, None)],
    );

    let (result, time) = measure(|| ops.link_in_bounds(&label, Reversed(EdgeIdT(0)), (13 * hour - 100, 16 * hour)));
    dbg!(&result);
    println!("Linking took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
}

#[test]
fn approximated_interval_merge() {
    let hour = MAX_BUCKETS / 24;

    let edge_profiles = vec![
        speed_profile_to_tt_profile(&vec![(0, 60), (12 * hour, 40), (MAX_BUCKETS, 60)], 1000),
        speed_profile_to_tt_profile(&vec![(0, 30), (8 * hour, 20), (16 * hour, 10), (MAX_BUCKETS, 30)], 500),
    ];

    let departures = edge_profiles
        .iter()
        .map(|val| val.iter().map(|&(a, _)| a).collect::<Vec<u32>>())
        .collect::<Vec<Vec<u32>>>();

    let travel_times = edge_profiles
        .iter()
        .map(|val| val.iter().map(|&(_, b)| b).collect::<Vec<u32>>())
        .collect::<Vec<Vec<u32>>>();

    let ops = TDCorridorIntervalPotentialOps::new(0, 1000, 6, &departures, &travel_times);

    let mut label1 = ApproximatedIntervalLabel::new(
        Some(43200000),
        vec![IntervalLabelEntry::new(105000, Some((80000, 5000))), IntervalLabelEntry::new(100000, None)],
    );

    let label2 = ApproximatedIntervalLabel::new(Some(28800000), vec![IntervalLabelEntry::new(55000, None), IntervalLabelEntry::new(85000, None)]);

    let result = ops.merge(&mut label1, label2);
    dbg!(&label1, &result);
}
