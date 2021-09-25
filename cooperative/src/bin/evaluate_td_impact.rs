use std::env;
use std::error::Error;

use cooperative::dijkstra::model::PathResult;
use cooperative::dijkstra::potentials::lowerbound_cch::init_cch_potential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::{generate_queries, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::report::measure;
use std::path::Path;

/// Evaluates the impact of time-dependent queries.
/// Comparison is provided by a more static setting with only 1 bucket per edge.
/// The evaluation itself takes place on the time-dependent graph.
/// Queries are generated randomly and either uniform, geometric or population-density-based.
///
/// Expected result: with a rising number of long-distance queries, the time-dependent graph
/// should outperform the static setting (in terms of travel times).
///
/// Additional parameters: <path_to_graph> <num_buckets> <num_queries> <query_mode = uniform>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, num_queries, query_type) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load static graph
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, 1, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init cch potential
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    // generate queries
    let queries = generate_queries(&graph, query_type, num_queries);

    // first run on time-independent graph
    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
    let paths = get_query_paths(&mut server, &queries);

    // repeat the same run with more buckets
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    println!("TD-Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
    let td_paths = get_query_paths(&mut server, &queries);

    // verify validity of results
    assert_eq!(paths.len(), td_paths.len(), "number of valid paths should be the same!");

    let path_dist_sum = sum_path_lengths(&paths, &server);
    let td_path_dist_sum = sum_path_lengths(&td_paths, &server);

    println!("-------------------------");
    println!("sum of distances (static): {}", path_dist_sum);
    println!("sum of distances (dynamic): {}", td_path_dist_sum);
    println!("-------------------------");

    //TODO write csv result for further comparison
    Ok(())
}

fn get_query_paths<Pot: TDPotential>(server: &mut CapacityServer<Pot>, queries: &Vec<TDQuery<Timestamp>>) -> Vec<PathResult> {
    queries
        .iter()
        .filter_map(|query| server.query(*query, true).map(|result| result.path))
        .collect::<Vec<PathResult>>()
}

fn sum_path_lengths<Pot: TDPotential>(paths: &Vec<PathResult>, server: &CapacityServer<Pot>) -> u128 {
    paths.iter().map(|path| server.path_distance(path) as u128).sum()
}

fn parse_args() -> Result<(String, u32, u32, QueryType), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_mode = parse_arg_optional(&mut args, QueryType::Uniform);

    Ok((graph_directory, num_buckets, num_queries, query_mode))
}
