use std::error::Error;
use std::path::Path;

use rust_road_router::io::Load;

pub mod io_graph;
pub mod io_td_graph;
pub mod parse_visum_graph;
pub mod population_grid;
pub mod io_raw_graph;
pub mod extract_scc;

pub fn load_coords(graph_directory: &Path) -> Result<(Vec<f32>, Vec<f32>), Box<dyn Error>> {
    let lon = Vec::load_from(graph_directory.join("longitude"))?;
    let lat = Vec::load_from(graph_directory.join("latitude"))?;

    Ok((lon, lat))
}