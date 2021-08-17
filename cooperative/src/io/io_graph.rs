use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::Weight;

use crate::graph::capacity_graph::{Capacity, CapacityGraph};
use crate::io::io_raw_graph::load_capacity_graph_raw;
use std::cmp::max;

pub fn load_capacity_graph(graph_directory: &Path, traffic_function: fn(Weight, Capacity, Capacity) -> Weight) -> Result<CapacityGraph, Box<dyn Error>> {
    let container = load_capacity_graph_raw(graph_directory)?;

    let distance = container.geo_distance.iter().map(|&dist| max(dist, 1)).collect::<Vec<u32>>();
    let freeflow_time = container.travel_time.iter().map(|&time| max(time, 1)).collect::<Vec<u32>>();

    Ok(CapacityGraph::new(
        container.first_out,
        container.head,
        freeflow_time,
        distance,
        container.capacity,
        traffic_function,
    ))
}
