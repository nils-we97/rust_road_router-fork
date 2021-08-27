/*use std::error::Error;
use std::path::Path;

use crate::graph::capacity_graph::Capacity;
use crate::graph::td_capacity_graph::TDCapacityGraph;
use crate::io::io_raw_graph::load_capacity_graph_raw;
use rust_road_router::datastr::graph::Weight;
use std::cmp::max;

pub fn load_td_capacity_graph(
    graph_directory: &Path,
    num_buckets: u32,
    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
) -> Result<TDCapacityGraph, Box<dyn Error>> {
    let container = load_capacity_graph_raw(graph_directory)?;

    let distance = container.geo_distance.iter().map(|&dist| max(dist, 1)).collect::<Vec<u32>>();

    let freeflow_time = container.travel_time.iter().map(|&time| max(time, 1)).collect::<Vec<u32>>();

    assert_eq!(freeflow_time.len(), container.head.len());
    assert_eq!(distance.len(), container.head.len());
    assert_eq!(container.capacity.len(), container.head.len());

    Ok(TDCapacityGraph::new(
        num_buckets,
        container.first_out,
        container.head,
        distance,
        freeflow_time,
        container.capacity,
        traffic_function,
    ))

    /*let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let freeflow_time = Vec::load_from(graph_directory.join("travel_time"))?;
    let distance: Vec<u32> = Vec::load_from(graph_directory.join("geo_distance"))?;
    let capacity = Vec::load_from(graph_directory.join("capacity"))
        .unwrap_or(vec![10; head.len()]);*/
}*/
