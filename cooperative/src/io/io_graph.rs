use std::cmp::max;
use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::{Graph, Weight};
use rust_road_router::io::{Load, Store};

use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::{Capacity, ExportableCapacity, Velocity};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

/// Loads and initializes a capacity graph with empty capacity buckets.
pub fn load_capacity_graph(
    graph_directory: &Path,
    num_buckets: u32,
    speed_function: fn(Velocity, Capacity, Capacity) -> Weight,
) -> Result<CapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::load_from(graph_directory.join("travel_time"))?;
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
        speed_function,
    ))
}

/// Loads and initializes a capacity graph with pre-filled capacity buckets.
pub fn load_used_capacity_graph(
    graph_directory: &Path,
    num_buckets: u32,
    speed_function: fn(Velocity, Capacity, Capacity) -> Weight,
    bucket_directory_ts: &str,
) -> Result<CapacityGraph, Box<dyn Error>> {
    let mut graph = load_capacity_graph(graph_directory, num_buckets, speed_function)?;

    let bucket_directory = graph_directory.join(bucket_directory_ts);

    let prefix_sum = Vec::<u32>::load_from(bucket_directory.join("prefix_sum"))?;
    let timestamps = Vec::<Timestamp>::load_from(bucket_directory.join("timestamps"))?;
    let capacities = Vec::<Capacity>::load_from(bucket_directory.join("values"))?;

    debug_assert_eq!(
        *prefix_sum.last().unwrap(),
        capacities.len() as u32,
        "Last prefix sum should be equal to total number of capacity buckets!"
    );
    debug_assert_eq!(
        prefix_sum.len(),
        graph.num_arcs() + 1,
        "Invalid length of bucket array: Expected {} buckets, found {}",
        graph.num_arcs(),
        prefix_sum.len() - 1
    );

    let mut buckets = vec![Vec::new(); graph.num_arcs()];
    for edge_id in 0..graph.num_arcs() {
        for bucket_id in prefix_sum[edge_id]..prefix_sum[edge_id + 1] {
            let bucket_id = bucket_id as usize;
            buckets[edge_id].push((timestamps[bucket_id], capacities[bucket_id]));
        }
    }

    graph.update_capacities(buckets);
    Ok(graph)
}

/// Stores the current capacity buckets
pub fn store_capacity_buckets(graph: &CapacityGraph, num_buckets: u32, directory: &Path) -> Result<(), Box<dyn Error>> {
    let capacities = graph.export_capacities();

    // create a prefix-sum over the entries (similar to the `first_out` array)
    let mut prefix_sum = vec![0u32];
    capacities.iter().for_each(|v| {
        let &last = prefix_sum.last().unwrap();
        prefix_sum.push(last + (v.len() as u32));
    });

    let (timestamps, capacities): (Vec<Timestamp>, Vec<Capacity>) = capacities.iter().flatten().cloned().unzip();

    let subfolder = format!("{}_{}", num_buckets, time::get_time().sec);
    let output_dir = directory.join(subfolder);
    std::fs::create_dir(&output_dir)?;

    prefix_sum.write_to(&output_dir.join("prefix_sum"))?;
    timestamps.write_to(&output_dir.join("timestamps"))?;
    capacities.write_to(&output_dir.join("values"))?;

    Ok(())
}
