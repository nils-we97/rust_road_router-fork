use crate::graph::Capacity;
use crate::io::io_coordinates::load_coords;
use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight};
use rust_road_router::io::{Load, Store};
use std::error::Error;
use std::path::Path;

pub mod extract_scc;
pub mod filter_invalid_nodes_and_edges;
pub mod remove_multi_edges;

pub struct CapacityGraphContainer {
    pub first_out: Vec<EdgeId>,
    pub head: Vec<NodeId>,
    pub geo_distance: Vec<Weight>,
    pub travel_time: Vec<Weight>,
    pub max_capacity: Vec<Capacity>,
    pub longitude: Vec<f32>,
    pub latitude: Vec<f32>,
}

pub fn load_raw_graph_data(graph_directory: &Path) -> Result<CapacityGraphContainer, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::<Weight>::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::<Weight>::load_from(graph_directory.join("travel_time"))?;
    let max_capacity = Vec::<Capacity>::load_from(graph_directory.join("capacity"))?;
    let (longitude, latitude) = load_coords(graph_directory)?;

    Ok(CapacityGraphContainer {
        first_out,
        head,
        geo_distance,
        travel_time,
        max_capacity,
        longitude,
        latitude,
    })
}

pub fn store_raw_data(raw_graph_data: &CapacityGraphContainer, output_directory: &Path) -> Result<(), Box<dyn Error>> {
    raw_graph_data.first_out.write_to(&output_directory.join("first_out"))?;
    raw_graph_data.head.write_to(&output_directory.join("head"))?;
    raw_graph_data.geo_distance.write_to(&output_directory.join("geo_distance"))?;
    raw_graph_data.travel_time.write_to(&output_directory.join("travel_time"))?;
    raw_graph_data.max_capacity.write_to(&output_directory.join("capacity"))?;
    raw_graph_data.longitude.write_to(&output_directory.join("longitude"))?;
    raw_graph_data.latitude.write_to(&output_directory.join("latitude"))?;

    Ok(())
}
