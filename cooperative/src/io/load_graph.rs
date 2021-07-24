use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::Weight;
use rust_road_router::io::Load;

use crate::graph::capacity_graph::{Capacity, CapacityGraph};

pub fn load_capacity_graph(
    graph_directory: &Path,
    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
) -> Result<CapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let weight = Vec::load_from(graph_directory.join("travel_time"))?;
    let capacity = vec![10; weight.len()];

    Ok(
        CapacityGraph::new(
            first_out,
            head,
            weight,
            capacity,
            traffic_function
        )
    )
}