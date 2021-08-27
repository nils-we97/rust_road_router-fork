use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::potentials::cch_potential_init::init_cch_potential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::{generate_queries, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::report::measure;

/// Compare runtimes of different speedup-techniques on the same graph with the same queries
///
/// Additional parameters: <path_to_graph> <num_buckets> <num_queries> <query_mode = uniform>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, num_queries, query_type) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load graph and node order
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (order, time) = measure(|| load_node_order(graph_directory).unwrap());
    println!("Node order loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // generate queries
    let queries = generate_queries(&graph, query_type, num_queries);

    // initialize potentials
    let cch_pot_data = init_cch_potential(&graph, order);
    let cch_potential = cch_pot_data.forward_potential();

    // 1. speedup technique: cch potentials
    let mut server = CapacityServer::new_with_potential(graph, cch_potential);

    let (_, time) = measure(|| {
        queries.iter().for_each(|query| {
            server.query(*query, true);
        })
    });
    println!(
        "CCH-Pot: {} queries took {} ms",
        num_queries,
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );

    // TODO more speedup techniques

    Ok(())
}

fn parse_args() -> Result<(String, u32, u32, QueryType), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_mode = parse_arg_optional(&mut args, QueryType::Uniform);

    Ok((graph_directory, num_buckets, num_queries, query_mode))
}
