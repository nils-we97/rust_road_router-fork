use std::path::Path;
use rust_road_router::io::Load;
use crate::graph::capacity_graph::{CapacityGraph, Capacity};
use std::error::Error;
use rust_road_router::datastr::graph::Weight;
use crate::graph::td_capacity_graph::TDCapacityGraph;

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
            traffic_function,
        )
    )
}

pub fn load_td_capacity_graph(
    graph_directory: &Path,
    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
) -> Result<TDCapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let mut freeflow_time = Vec::load_from(graph_directory.join("travel_time"))?;
    let distance = Vec::load_from(graph_directory.join("geo_distance"))?;
    let capacity = vec![10; head.len()];

    for i in 0..freeflow_time.len() {
        if freeflow_time[i] == 0 {
            // freeflow times must NOT be zero! Otherwise speeds can't be calculated!
            freeflow_time[i] = 1;
        }
    }

    Ok(
        TDCapacityGraph::new(
            10,
            first_out,
            head,
            distance,
            freeflow_time,
            capacity,
            traffic_function
        )
    )
}

pub fn load_coords(graph_directory: &Path) -> Result<(Vec<f32>, Vec<f32>), Box<dyn Error>> {
    let lon = Vec::load_from(graph_directory.join("longitude"))?;
    let lat = Vec::load_from(graph_directory.join("latitude"))?;

    Ok((lon, lat))
}