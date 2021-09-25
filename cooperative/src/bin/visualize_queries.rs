use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::potentials::lowerbound_cch::init_cch_potential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::{generate_queries, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_coordinates::load_coords;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use cooperative::util::query_path_visualization::print_path_coords;
use rust_road_router::report::measure;

/// Runs a specified number of random queries on a given graph.
/// Prints the resulting paths for further visualization.
///
/// Additional parameters: <path_to_graph> <query_type = uniform> <num_buckets = 50> <num_queries = 100>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, query_type, num_buckets, num_queries) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load graph
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load coords
    let ((lon, lat), time) = measure(|| load_coords(graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init cch potential
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());

    // generate and run queries, print resulting path coordinates
    generate_queries(server.borrow_graph(), query_type, num_queries)
        .iter()
        .filter_map(|query| server.query(*query, true).map(|result| result.path))
        .for_each(|path| print_path_coords(&path.node_path, &lat, &lon));

    Ok(())
}

fn parse_args() -> Result<(String, QueryType, u32, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_type = parse_arg_optional(&mut args, QueryType::Uniform);
    let num_buckets = parse_arg_optional(&mut args, 50u32);
    let num_queries = parse_arg_optional(&mut args, 100u32);

    Ok((graph_directory, query_type, num_buckets, num_queries))
}
