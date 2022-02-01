use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::complete_balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::{load_capacity_graph, store_speed_buckets};
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use std::env;
use std::error::Error;
use std::path::Path;
use std::time::Instant;

/// Creates a basic traffic load on a capacity graph
///
/// Runs a given query set and stores the resulting speed buckets.
/// The resulting data can be used in the actual query phase to support predictions of future traffic conditions
///
/// Additional parameters: <path_to_graph> <path_to_queries> <output_path> <num_buckets>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, output_directory, num_buckets) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);
    let speed_path = graph_path.join("speeds");

    if !speed_path.exists() {
        std::fs::create_dir(&speed_path)?;
    }
    let output_path = speed_path.join(&output_directory);
    if !output_path.exists() {
        std::fs::create_dir(&output_path)?;
    }

    // load graph and queries
    let graph = load_capacity_graph(&graph_path, num_buckets, BPRTrafficFunction::default())?;
    let mut queries = load_queries(&query_path)?;
    permutate_queries(&mut queries);

    // init potential and server
    let order = load_node_order(&graph_path)?;
    let cch = CCH::fix_order_and_build(&graph, order);
    let interval_pattern = complete_balanced_interval_pattern();
    let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &interval_pattern, 20);
    let mut server = CapacityServer::new(graph, customized);

    let mut time = Instant::now();

    // execute queries
    for idx in 0..queries.len() {
        if (idx + 1) % 10000 == 0 {
            println!(
                "Finished {} of {} queries - last step took {}s",
                idx + 1,
                queries.len(),
                time.elapsed().as_secs_f64()
            );
            time = Instant::now();
        }

        server.query(&queries[idx], true);

        // customize graph regularly
        if (idx + 1) % 50000 == 0 {
            server.customize(&interval_pattern, 20);
        }

        if !server.result_valid() || !server.update_valid() {
            // re-customization of upper bounds
            println!("-- {} - potential update after {} steps", num_buckets, idx + 1);
            server.customize_upper_bound();
        }
    }

    println!("Finished queries, starting to extract and store the speed buckets..");
    store_speed_buckets(&output_path, server.borrow_graph())
}

fn parse_args() -> Result<(String, String, String, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let output_directory = parse_arg_required(&mut args, "Output Directory")?;
    let num_buckets = parse_arg_required(&mut args, "Num Buckets")?;

    Ok((graph_directory, query_directory, output_directory, num_buckets))
}
