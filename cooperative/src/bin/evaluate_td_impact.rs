use std::env;
use std::error::Error;

use cooperative::dijkstra::model::PathResult;
use cooperative::dijkstra::potentials::init_cch_potential::init_cch_potential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use std::path::Path;

/// Evaluates the impact of time-dependence on a given set of queries.
/// Comparison is provided by a more static setting with only 1 bucket per edge.
/// The evaluation itself takes place on the time-dependent graph, with several steps (100/200/300/400/600 buckets)
///
/// Expected result: with a rising number of long-distance queries, the time-dependent graph
/// should outperform the static setting (in terms of travel times).
///
/// Additional parameters: <path_to_graph> <path_to_queries>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory) = parse_args()?;
    let path = Path::new(&graph_directory);

    // load queries
    let query_path = path.join("queries").join(query_directory);
    let queries = load_queries(&query_path)?;

    let graph = load_capacity_graph(&path, 1, bpr_speed_function).unwrap();

    // init cch potential
    let order = load_node_order(&path)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    let mut path_results = Vec::new();
    let bucket_sizes = [1, 100, 200, 300, 400, 600];

    // separate treatment for first round
    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
    path_results.push(get_query_paths(&mut server, &queries));

    // determine paths for all remaining bucket settings
    for &num_buckets in &bucket_sizes[1..] {
        let graph = load_capacity_graph(&path, num_buckets, bpr_speed_function).unwrap();
        server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
        path_results.push(get_query_paths(&mut server, &queries));
    }

    // evaluation based on struct with highest number of buckets
    debug_assert_eq!(bucket_sizes.iter().max(), bucket_sizes.last());

    path_results.iter().enumerate().for_each(|(idx, paths)| {
        let path_dist_sum = sum_path_lengths(paths, &server);
        println!(
            "Sum of path lengths for {} buckets: {} ({} valid paths)",
            bucket_sizes[idx],
            path_dist_sum,
            paths.len()
        );
    });

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

fn parse_args() -> Result<(String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;

    Ok((graph_directory, query_directory))
}
