use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::Weight;
use rust_road_router::io::Load;

use crate::graph::capacity_graph::Capacity;
use crate::graph::td_capacity_graph::TDCapacityGraph;
use std::cmp::max;

pub fn load_td_capacity_graph(
    graph_directory: &Path,
    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
) -> Result<TDCapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let freeflow_time = Vec::load_from(graph_directory.join("travel_time"))?;
    let distance: Vec<u32> = Vec::load_from(graph_directory.join("geo_distance"))?;
    let capacity = Vec::load_from(graph_directory.join("capacity"))
        .unwrap_or(vec![10; head.len()]);

    let num_edges = head.len();

    let distance = distance.iter().map(|&dist| max(dist, 1)).collect::<Vec<u32>>();
    let freeflow_time = freeflow_time.iter().map(|&time| max(time, 1)).collect::<Vec<u32>>();

    assert_eq!(freeflow_time.len(), num_edges);
    assert_eq!(distance.len(), num_edges);
    assert_eq!(capacity.len(), num_edges);

    Ok(
        TDCapacityGraph::new(
            10,
            first_out,
            head,
            distance,
            freeflow_time,
            capacity,
            traffic_function,
        )
    )
}