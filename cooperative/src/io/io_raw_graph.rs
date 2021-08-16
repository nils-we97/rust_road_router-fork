use crate::graph::capacity_graph::Capacity;
use crate::io::load_coords;
use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight};
use rust_road_router::io::{Load, Store};
use std::error::Error;
use std::path::Path;

pub struct RawCapacityGraphContainer {
    pub first_out: Vec<EdgeId>,
    pub head: Vec<NodeId>,
    pub geo_distance: Vec<Weight>,
    pub travel_time: Vec<Weight>,
    pub capacity: Vec<Capacity>,
    pub longitude: Vec<f32>,
    pub latitude: Vec<f32>,
}

pub fn load_capacity_graph_raw(graph_directory: &Path) -> Result<RawCapacityGraphContainer, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::load_from(graph_directory.join("travel_time"))?;
    let capacity = Vec::load_from(graph_directory.join("capacity"))?;

    let (longitude, latitude) = load_coords(graph_directory)?;

    Ok(RawCapacityGraphContainer {
        first_out,
        head,
        geo_distance,
        travel_time,
        capacity,
        longitude,
        latitude,
    })
}

pub fn store_capacity_graph_raw(container: &RawCapacityGraphContainer, graph_directory: &Path) -> Result<(), Box<dyn Error>> {
    container.first_out.write_to(&graph_directory.join("first_out"))?;
    container.head.write_to(&graph_directory.join("head"))?;
    container.geo_distance.write_to(&graph_directory.join("geo_distance"))?;
    container.travel_time.write_to(&graph_directory.join("travel_time"))?;
    container.capacity.write_to(&graph_directory.join("capacity"))?;
    container.longitude.write_to(&graph_directory.join("longitude"))?;
    container.latitude.write_to(&graph_directory.join("latitude"))?;

    Ok(())
}
