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
    let capacity = Vec::load_from(graph_directory.join("capacity"))
        .unwrap_or(vec![10; head.len()]);

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

/*pub fn store_capacity_graph(
    graph_directory: &Path,
    graph: &CapacityGraph,
    file_prefix: &str
) -> Result<(), Box<dyn Error>> {

    let (first_out, head, weight, capacity) = graph.decompose();

    let path = format!("{}first_out", file_prefix);
    first_out.write_to(&graph_directory.join(path));

    let path = format!("{}head", file_prefix);
    head.write_to(&graph_directory.join(path));

    let path = format!("{}weight", file_prefix);
    weight.write_to(&graph_directory.join(path));

    let path = format!("{}capacity", file_prefix);
    capacity.write_to(&graph_directory.join(path));

    Ok(())
}*/