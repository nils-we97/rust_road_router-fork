use cooperative::experiments::queries::departure_distributions::{
    ConstantDeparture, DepartureDistribution, NormalDeparture, RushHourDeparture, UniformDeparture,
};
use cooperative::experiments::queries::population_density_based::{
    generate_geometric_population_density_based_queries, generate_uniform_population_density_based_queries,
};
use cooperative::experiments::queries::random_geometric::generate_random_geometric_queries;
use cooperative::experiments::queries::random_uniform::generate_random_uniform_queries;
use cooperative::experiments::queries::QueryType;
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_coordinates::load_coords;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_population_grid::load_population_grid;
use cooperative::io::io_queries::store_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::datastr::graph::{FirstOutGraph, Graph};
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
    let graph = load_capacity_graph(graph_directory, 1, bpr_speed_function)?;

    let queries = match query_type {
        QueryType::Uniform => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, UniformDeparture::new()),
        QueryType::UniformRushHourDep => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, RushHourDeparture::new()),
        QueryType::UniformNormalDep => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, NormalDeparture::new()),
        QueryType::Geometric | QueryType::GeometricRushHourDep => {
            let graph_with_distance_metric = FirstOutGraph::new(graph.first_out(), graph.head(), graph.distance());

            match query_type {
                QueryType::Geometric => generate_random_geometric_queries(&graph_with_distance_metric, num_queries, UniformDeparture::new()),
                QueryType::GeometricRushHourDep => generate_random_geometric_queries(&graph_with_distance_metric, num_queries, RushHourDeparture::new()),
                _ => unimplemented!(),
            }
        }
        _ => {
            // for population queries, we have to use some additional data
            let population_path: String = parse_arg_required(&mut remaining_args, "population grid directory")?;
            let population_directory = Path::new(&population_path);

            let (longitude, latitude) = load_coords(graph_directory)?;
            let (grid_tree, grid_population) = load_population_grid(population_directory)?;

            // TODO implementation for geometric queries
            match query_type {
                QueryType::PopulationUniform => {
                    generate_uniform_population_density_based_queries(&longitude, &latitude, &grid_tree, &grid_population, num_queries, UniformDeparture::new())
                }
                QueryType::PopulationUniformConstantDep => generate_uniform_population_density_based_queries(
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    num_queries,
                    ConstantDeparture::new(),
                ),
                QueryType::PopulationGeometric => generate_geometric_population_density_based_queries(
                    &graph,
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    num_queries,
                    RushHourDeparture::new(),
                ),
                /*QueryType::PopulationGeometric => {}
                QueryType::PopulationGeometricConstantDep => {}*/
                _ => unimplemented!(),
            }
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
