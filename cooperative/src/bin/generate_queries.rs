use cooperative::experiments::queries::{generate_population_queries, generate_queries, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_coordinates::load_coords;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_population_grid::load_population_grid;
use cooperative::io::io_queries::store_queries;
use cooperative::util::cli_args::parse_arg_required;
use std::env;
use std::error::Error;
use std::path::Path;

/// Generate random queries and store them in a given directory
///
/// First parameters: <path_to_graph> <num_queries> <query_type>
/// Additional parameters, depending on `query_type`:
/// uniform/geometric: ---
/// population-grid-based: <path_to_population_grid_file>
///
/// Results will be written to directory <path_to_graph>/queries/<timestamp>/
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_queries, query_type, mut remaining_args) = parse_required_args()?;
    let graph_directory = Path::new(&path);

    let queries = match query_type {
        QueryType::Uniform | QueryType::UniformConstantDep | QueryType::Geometric | QueryType::GeometricConstantDep => {
            let graph = load_capacity_graph(graph_directory, 1, bpr_speed_function)?;
            generate_queries(&graph, query_type, num_queries)
        }
        QueryType::PopulationUniform | QueryType::PopulationUniformConstantDep | QueryType::PopulationGeometric | QueryType::PopulationGeometricConstantDep => {
            let population_path: String = parse_arg_required(&mut remaining_args, "population grid directory")?;
            let population_directory = Path::new(&population_path);

            let graph = load_capacity_graph(graph_directory, 1, bpr_speed_function)?;
            let (longitude, latitude) = load_coords(graph_directory)?;
            let (grid_tree, grid_population) = load_population_grid(population_directory)?;

            generate_population_queries(&graph, query_type, num_queries, &grid_tree, &grid_population, &longitude, &latitude)
        }
    };

    // check if subfolder `queries` exists
    let query_directory = graph_directory.join("queries");
    if !query_directory.exists() {
        std::fs::create_dir(query_directory)?;
    }

    // add new subfolder in `queries`
    let subfolder = format!("{}_{}", num_queries, time::get_time().sec);
    let output_dir = graph_directory.join("queries").join(subfolder);
    std::fs::create_dir(&output_dir)?;

    store_queries(&queries, &output_dir)?;

    println!("Wrote {} queries to {}", num_queries, output_dir.display());

    Ok(())
}

fn parse_required_args() -> Result<(String, u32, QueryType, impl Iterator<Item = String>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_type = parse_arg_required(&mut args, "query type")?;

    Ok((graph_directory, num_queries, query_type, args))
}
