use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::Weight;

use crate::graph::capacity_graph::{Capacity, CapacityGraph};
use crate::io::io_raw_graph::load_capacity_graph_raw;

pub fn load_capacity_graph(
    graph_directory: &Path,
    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
) -> Result<CapacityGraph, Box<dyn Error>> {
    let container = load_capacity_graph_raw(graph_directory)?;

    Ok(
        CapacityGraph::new(
            container.first_out,
            container.head,
            container.travel_time,
            container.capacity,
            traffic_function,
        )
    )
}