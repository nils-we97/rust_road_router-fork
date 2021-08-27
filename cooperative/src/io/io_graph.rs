use std::cmp::max;
use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::Weight;
use rust_road_router::io::Load;

use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::{Capacity, Velocity};

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
