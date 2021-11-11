use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::elimination_tree::approximated_periodic_ttf::customized::CustomizedApproximatedPeriodicTTF;
use cooperative::dijkstra::elimination_tree::multi_level_buckets::customized::CustomizedMultiLevels;
use cooperative::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use cooperative::dijkstra::potentials::multi_level_interval_potential::CCHMultiLevelIntervalPotential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_used_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::Graph;
use rust_road_router::report::measure;
use std::ops::Add;

/// Compare runtimes of different potentials on the same graph with the same queries
/// Initializes a `CapacityGraph` with a given benchmark of capacities,
/// then executes a set of queries (without updates) on several potentials
///
/// Additional parameters: <path_to_graph> <num_buckets> <benchmark_capacities> <query_path>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, capacity_directory, query_directory) = parse_args()?;
    let graph_directory = Path::new(&path);

    // load graph
    let (graph, time) = measure(|| load_used_capacity_graph(graph_directory, num_buckets, bpr_speed_function, &capacity_directory).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load queries
    let queries = load_queries(&graph_directory.join("queries").join(query_directory))?;

    // load node order, build CCH
    let order = load_node_order(graph_directory)?;
    let cch = CCH::fix_order_and_build(&graph, order);

    // ----------------------------------------------------------------------------- //
    // 1st potential: CCH Lowerbound Potential (Benchmark)
    let cch_pot_data = CCHPotData::new(&cch, &graph);
    let cch_lowerbound_pot = cch_pot_data.forward_potential();
    let mut server = CapacityServer::new_with_potential(graph, cch_lowerbound_pot);

    execute_queries(&mut server, &queries, "CCH Lowerbound Potential");
    let (graph, _) = server.decompose();
    drop(cch_pot_data); // reduce memory consumption

    // ----------------------------------------------------------------------------- //
    // 2nd potential: Multi-Level-Bucket Potential
    let customized_multi_levels = CustomizedMultiLevels::new(
        &cch,
        graph.departure(),
        graph.travel_time(),
        &vec![86_400_000 / 12, 86_400_000 / 48],
        graph.num_arcs() as u64 * 10,
    );
    let multi_level_bucket_pot = CCHMultiLevelIntervalPotential::new_forward(&customized_multi_levels, 2);
    let mut server = CapacityServer::new_with_potential(graph, multi_level_bucket_pot);

    execute_queries(&mut server, &queries, "Multi-Level-Bucket Potential");
    let (graph, _) = server.decompose();
    drop(customized_multi_levels);

    // ----------------------------------------------------------------------------- //
    // 3rd potential: Corridor-Lowerbound Potential
    let customized_corridor_lowerbound = CustomizedApproximatedPeriodicTTF::new(&cch, graph.departure(), graph.travel_time(), 200, 48);
    let corridor_lowerbound_pot = CorridorLowerboundPotential::new(&customized_corridor_lowerbound);
    let mut server = CapacityServer::new_with_potential(graph, corridor_lowerbound_pot);

    execute_queries(&mut server, &queries, "Corridor Lowerbound Potential");
    Ok(())
}

fn parse_args() -> Result<(String, u32, String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let capacity_directory: String = parse_arg_required(&mut args, "path to capacities")?;
    let query_directory: String = parse_arg_required(&mut args, "path to queries")?;

    Ok((graph_directory, num_buckets, capacity_directory, query_directory))
}

fn execute_queries<Pot: TDPotential>(server: &mut CapacityServer<Pot>, queries: &Vec<TDQuery<u32>>, pot_name: &str) {
    let mut sum_distances = 0u64;

    let mut time_total = time::Duration::zero();
    let mut time_pot_init = time::Duration::zero();
    let mut time_query = time::Duration::zero();
    let mut time_updates = time::Duration::zero();
    let mut num_queue_pops = 0u64;
    let mut num_relaxed_arcs = 0u64;

    queries.iter().enumerate().for_each(|(idx, &query)| {
        let result = server.query_measured(query, false);

        time_total = time_total
            .add(result.distance_result.time_query)
            .add(result.distance_result.time_potential)
            .add(result.update_result.time_ttf)
            .add(result.update_result.time_bucket_updates);

        time_query = time_query.add(result.distance_result.time_query);
        time_pot_init = time_pot_init.add(result.distance_result.time_potential);
        time_updates = time_updates.add(result.update_result.time_ttf.add(result.update_result.time_bucket_updates));

        num_queue_pops += result.distance_result.num_queue_pops as u64;
        num_relaxed_arcs += result.distance_result.num_relaxed_arcs as u64;

        sum_distances += result.distance_result.distance.map(|val| val as u64).unwrap_or(0);

        if (idx + 1) % 10000 == 0 {
            println!("Finished {}/{} queries", idx + 1, queries.len());
        }
    });

    println!("-----------------------------");
    println!("Result for {}:", pot_name);
    println!(
        "Total runtime: {} ms (potential init: {} ms, queries: {} ms, update: {} ms)",
        time_total.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_pot_init.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_query.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_updates.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "Query statistics: {} relaxed arcs (avg: {}), {} queue pops (avg: {})",
        num_relaxed_arcs,
        num_relaxed_arcs / queries.len() as u64,
        num_queue_pops,
        num_queue_pops / queries.len() as u64
    );
    println!("Sum of distances: {}", sum_distances);
    println!("-----------------------------");
}
