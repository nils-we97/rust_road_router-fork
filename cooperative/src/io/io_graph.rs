use std::cmp::max;
use std::error::Error;
use std::path::Path;

use rust_road_router::io::{Load, Store};

use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::edge_buckets::SpeedBuckets;
use crate::graph::traffic_functions::BPRTrafficFunction;

/// Loads and initializes a capacity graph with empty capacity buckets.
pub fn load_capacity_graph(graph_directory: &Path, num_buckets: u32, traffic_function: BPRTrafficFunction) -> Result<CapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::<u32>::load_from(graph_directory.join("travel_time"))?;
    let capacity = Vec::load_from(graph_directory.join("capacity"))?;

    // modify distance and travel_time to avoid divisions by zero
    let distance = geo_distance.iter().map(|&dist| max(dist, 1)).collect::<Vec<u32>>();
    let freeflow_time = travel_time.iter().map(|&time| max(time, 1)).collect::<Vec<u32>>();

    Ok(CapacityGraph::new(
        num_buckets,
        first_out,
        head,
        distance,
        freeflow_time,
        capacity,
        traffic_function,
    ))
}

pub fn load_used_speed_profiles(directory: &Path) -> Result<Vec<SpeedBuckets>, Box<dyn Error>> {
    let prefix_sum = Vec::<u32>::load_from(&directory.join("prefix_sum"))?;
    let timestamps = Vec::<u32>::load_from(&directory.join("timestamps"))?;
    let speeds = Vec::<u32>::load_from(&directory.join("speeds"))?;

    let mut ret = Vec::with_capacity(prefix_sum.len() - 1);

    for a in prefix_sum.windows(2) {
        if a[0] == a[1] {
            ret.push(SpeedBuckets::Unused);
        } else {
            let inner = timestamps[a[0] as usize..a[1] as usize]
                .iter()
                .zip(speeds[a[0] as usize..a[1] as usize].iter())
                .map(|(&a, &b)| (a, b))
                .collect::<Vec<(u32, u32)>>();
            ret.push(SpeedBuckets::Used(inner));
        }
    }

    Ok(ret)
}

pub fn store_speed_buckets(directory: &Path, graph: &CapacityGraph) -> Result<(), Box<dyn Error>> {
    let mut prefix_sum = vec![0];
    let speed_buckets = graph.export_speeds();

    for speed_bucket in &speed_buckets {
        prefix_sum.push(*prefix_sum.last().unwrap() + speed_bucket.len() as u32);
    }

    let (timestamps, speeds): (Vec<u32>, Vec<u32>) = speed_buckets.iter().flatten().cloned().unzip();

    prefix_sum.write_to(&directory.join("prefix_sum"))?;
    timestamps.write_to(&directory.join("timestamps"))?;
    speeds.write_to(&directory.join("speeds"))?;

    Ok(())
}
