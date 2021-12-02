use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use rust_road_router::algo::customizable_contraction_hierarchy::DirectedCCH;
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
    println!(
        "Loaded upward/downward intervals in {} ms",
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );

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
    println!("Loaded upward/downward bounds in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (cch, time) = measure(|| DirectedCCH::reconstruct_from(&directory.join("cch")).unwrap());
    println!("Reconstructed directed CCH in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

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
