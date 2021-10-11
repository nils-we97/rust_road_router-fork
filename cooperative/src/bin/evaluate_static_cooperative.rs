use cooperative::dijkstra::model::PathResult;
use cooperative::dijkstra::potentials::cch::init_cch_potential;
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
use std::env;
use std::error::Error;
use std::path::Path;

/// Evaluate the difference between static (no updates) and cooperative routing.
/// In the static setting, no updates take place after calculating the routes.
/// The evaluation takes place on the frequently updated graph.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// Additional parameters: <path_to_graph> <num_buckets> <num_queries> <query_mode = uniform>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, num_queries, query_type) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load graph
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init cch potential
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    // generate queries
    let queries = generate_queries(&graph, query_type, num_queries);

    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());

    println!("Starting queries...");

    // generate paths for static (without updates) and cooperative setting
    let static_paths = get_query_paths(&mut server, &queries, false);
    let dynamic_paths = get_query_paths(&mut server, &queries, true);

    assert_eq!(static_paths.len(), dynamic_paths.len(), "number of paths should be the same!");

    let total_dist_static = sum_path_lengths(&static_paths, &server);
    let total_dist_dynamic = sum_path_lengths(&dynamic_paths, &server);

    println!("-------------------------");
    println!("sum of distances (static): {}", total_dist_static);
    println!("sum of distances (dynamic): {}", total_dist_dynamic);
    println!("-------------------------");

    //TODO write csv result for further comparison

    Ok(())
}

fn get_query_paths<Pot: TDPotential>(server: &mut CapacityServer<Pot>, queries: &Vec<TDQuery<Timestamp>>, update: bool) -> Vec<PathResult> {
    queries
        .iter()
        .filter_map(|query| server.query(*query, update).map(|result| result.path))
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
    let query_type: QueryType = parse_arg_optional(&mut args, QueryType::Uniform);

    Ok((graph_directory, num_buckets, num_queries, query_type))
}
