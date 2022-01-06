use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::potentials::init_cch_potential::init_cch_potential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::io::io_coordinates::load_coords;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use cooperative::util::query_path_visualization::print_path_coords;
use rust_road_router::report::measure;

/// Runs a given set of pre-generated queries on a given graph.
/// Prints the resulting paths for further visualization.
///
/// Additional parameters: <path_to_graph> <query_directory> <num_buckets = 50>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, query_directory, num_buckets) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load graph
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_traffic_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load coords
    let ((lon, lat), time) = measure(|| load_coords(graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load queries
    let queries = load_queries(&graph_directory.join("queries").join(query_directory))?;

    // init cch potential
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());

    // generate and run queries, print resulting path coordinates
    queries
        .iter()
        .filter_map(|query| server.query(*query, true).map(|result| result.path))
        .for_each(|path| print_path_coords(&path.node_path, &lat, &lon));

    Ok(())
}

fn parse_args() -> Result<(String, String, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory: String = parse_arg_required(&mut args, "Query Directory")?;
    let num_buckets = parse_arg_optional(&mut args, 50u32);

    Ok((graph_directory, query_directory, num_buckets))
}
