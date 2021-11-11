use cooperative::experiments::queries::departure_distributions::{DepartureDistribution, RushHourDeparture, UniformDeparture};
use cooperative::experiments::queries::random_geometric::generate_random_geometric_queries;
use cooperative::experiments::queries::random_uniform::generate_random_uniform_queries;
use cooperative::experiments::queries::QueryType;
use cooperative::io::io_queries::store_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::datastr::graph::time_dependent::TDGraph;
use rust_road_router::datastr::graph::{FirstOutGraph, Graph};
use rust_road_router::io::{Load, Reconstruct};
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
    let (path, num_queries, query_type) = parse_required_args()?;
    let graph_directory = Path::new(&path);

    let graph = TDGraph::reconstruct_from(&graph_directory)?;
    let lower_bound = Vec::<u32>::load_from(&graph_directory.join("lower_bound"))?;

    let queries = match query_type {
        QueryType::Uniform => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, UniformDeparture::new()),
        QueryType::UniformRushHourDep => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, RushHourDeparture::new()),
        QueryType::Geometric | QueryType::GeometricRushHourDep => {
            let lower_bound_graph = FirstOutGraph::new(graph.first_out(), graph.head(), &lower_bound[..]);

            match query_type {
                QueryType::Geometric => generate_random_geometric_queries(&lower_bound_graph, num_queries, UniformDeparture::new()),
                QueryType::GeometricRushHourDep => generate_random_geometric_queries(&lower_bound_graph, num_queries, RushHourDeparture::new()),
                _ => unimplemented!(),
            }
        }
        _ => panic!("Feature is not implemented yet!"),
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

fn parse_required_args() -> Result<(String, u32, QueryType), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_type = parse_arg_required(&mut args, "query type")?;

    Ok((graph_directory, num_queries, query_type))
}
