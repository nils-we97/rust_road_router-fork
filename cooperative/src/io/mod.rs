use std::error::Error;
use std::path::Path;

use rust_road_router::io::Load;

pub mod extract_scc;
pub mod io_graph;
pub mod io_node_order;
pub mod io_population_grid;

pub fn load_coords(graph_directory: &Path) -> Result<(Vec<f32>, Vec<f32>), Box<dyn Error>> {
    let lon = Vec::load_from(graph_directory.join("longitude"))?;
    let lat = Vec::load_from(graph_directory.join("latitude"))?;

    Ok((lon, lat))
}
