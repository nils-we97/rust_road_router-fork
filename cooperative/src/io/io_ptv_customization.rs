use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use crate::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use crate::dijkstra::potentials::multi_metric_potential::metric_reduction::MetricEntry;
use rust_road_router::algo::customizable_contraction_hierarchy::{DirectedCCH, CCH};
use rust_road_router::datastr::graph::Graph;
use rust_road_router::io::{Deconstruct, Load, Reconstruct, Store};
use rust_road_router::report::measure;
use std::error::Error;
use std::path::Path;

pub fn load_interval_minima(directory: &Path) -> Result<CustomizedApproximatedPeriodicTTF<DirectedCCH>, Box<dyn Error>> {
    let num_intervals = *Vec::<u32>::load_from(&directory.join("num_intervals")).unwrap().first().unwrap();
    println!("Number of intervals: {}", num_intervals);

    let ((downward_intervals, upward_intervals), time) = measure(|| {
        (
            Vec::<u32>::load_from(&directory.join("downward_intervals")).unwrap(),
            Vec::<u32>::load_from(&directory.join("upward_intervals")).unwrap(),
        )
    });
    println!("Loaded upward/downward intervals in {} ms", time.as_secs_f64() * 1000.0);

    let ((upward_bounds, downward_bounds), time) = measure(|| {
        let upward_lower = Vec::<u32>::load_from(&directory.join("upward_lower")).unwrap();
        let upward_upper = Vec::<u32>::load_from(&directory.join("upward_upper")).unwrap();
        let upward_bounds = upward_lower.iter().zip(upward_upper.iter()).map(|(&a, &b)| (a, b)).collect::<Vec<(u32, u32)>>();

        let downward_lower = Vec::<u32>::load_from(&directory.join("downward_lower")).unwrap();
        let downward_upper = Vec::<u32>::load_from(&directory.join("downward_upper")).unwrap();
        let downward_bounds = downward_lower
            .iter()
            .zip(downward_upper.iter())
            .map(|(&a, &b)| (a, b))
            .collect::<Vec<(u32, u32)>>();

        (upward_bounds, downward_bounds)
    });
    println!("Loaded upward/downward bounds in {} ms", time.as_secs_f64() * 1000.0);

    let (cch, time) = measure(|| DirectedCCH::reconstruct_from(&directory.join("cch")).unwrap());
    println!("Reconstructed directed CCH in {} ms", time.as_secs_f64() * 1000.0);

    Ok(CustomizedApproximatedPeriodicTTF {
        cch,
        upward_intervals,
        downward_intervals,
        upward_bounds,
        downward_bounds,
        num_intervals,
    })
}

pub fn store_interval_minima(directory: &Path, customized: &CustomizedApproximatedPeriodicTTF<DirectedCCH>) -> Result<(), Box<dyn Error>> {
    if !directory.exists() {
        std::fs::create_dir(directory)?;
    }

    let cch_directory = directory.join("cch");
    if !cch_directory.exists() {
        std::fs::create_dir(&cch_directory)?;
    }

    customized.cch.deconstruct_to(&cch_directory)?;
    customized.downward_intervals.write_to(&directory.join("downward_intervals"))?;
    customized.upward_intervals.write_to(&directory.join("upward_intervals"))?;

    let (upward_lower, upward_upper): (Vec<u32>, Vec<u32>) = customized.upward_bounds.iter().map(|&(a, b)| (a, b)).unzip();
    let (downward_lower, downward_upper): (Vec<u32>, Vec<u32>) = customized.downward_bounds.iter().map(|&(a, b)| (a, b)).unzip();

    upward_lower.write_to(&directory.join("upward_lower"))?;
    upward_upper.write_to(&directory.join("upward_upper"))?;
    downward_lower.write_to(&directory.join("downward_lower"))?;
    downward_upper.write_to(&directory.join("downward_upper"))?;

    vec![customized.num_intervals].write_to(&directory.join("num_intervals"))?;

    Ok(())
}

/* ----------------------------------------------------------------------------------------*/

pub fn load_multiple_metrics<'a>(directory: &Path, cch: &'a CCH) -> Result<CustomizedMultiMetrics<'a>, Box<dyn Error>> {
    let upward = Vec::<u32>::load_from(&directory.join("upward_weights"))?;
    let downward = Vec::<u32>::load_from(&directory.join("downward_weights"))?;

    let num_metrics = upward.len() / cch.num_arcs();

    assert_eq!(downward.len(), upward.len());
    assert_eq!(upward.len() % cch.num_arcs(), 0);

    let metric_start = Vec::<u32>::load_from(&directory.join("metric_start"))?;
    let metric_end = Vec::<u32>::load_from(&directory.join("metric_end"))?;
    let metric_ids = Vec::<u32>::load_from(&directory.join("metric_ids"))?;

    assert_eq!(metric_start.len(), metric_end.len());
    assert_eq!(metric_start.len(), metric_ids.len());

    let metric_entries = metric_start
        .iter()
        .zip(metric_end.iter())
        .zip(metric_ids.iter())
        .map(|((&start, &end), &id)| MetricEntry::new(start, end, id as usize))
        .collect::<Vec<MetricEntry>>();

    Ok(CustomizedMultiMetrics {
        cch,
        upward,
        downward,
        metric_entries,
        num_metrics,
    })
}

pub fn store_multiple_metrics(directory: &Path, customized: &CustomizedMultiMetrics) -> Result<(), Box<dyn Error>> {
    let metric_start = customized.metric_entries.iter().map(|m| m.start).collect::<Vec<u32>>();
    let metric_end = customized.metric_entries.iter().map(|m| m.end).collect::<Vec<u32>>();
    let metric_ids = customized.metric_entries.iter().map(|m| m.metric_id as u32).collect::<Vec<u32>>();

    metric_start.write_to(&directory.join("metric_start"))?;
    metric_end.write_to(&directory.join("metric_end"))?;
    metric_ids.write_to(&directory.join("metric_ids"))?;

    customized.upward.write_to(&directory.join("upward_weights"))?;
    customized.downward.write_to(&directory.join("downward_weights"))?;

    Ok(())
}
