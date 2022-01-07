use cooperative::dijkstra::model::PathResult;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::ch_potentials::{BorrowedCCHPot, CCHPotData};
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::OwnedGraph;
use rust_road_router::io::{Load, Store};
use std::env;
use std::error::Error;
use std::path::Path;
use std::process::exit;
use std::str::FromStr;

/// Evaluate the difference between static (no updates) and cooperative routing.
/// In the static setting, no updates take place after calculating the routes.
/// The evaluation takes place on the frequently updated graph.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// Additional parameters: <path_to_graph> <path_to_queries> <query_breakpoints, comma-separated> <query_buckets=50,200,600>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, query_breakpoints, graph_num_buckets) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // read statistic results
    if query_path.join("perf_stats_num_buckets").exists() {
        let s_num_buckets = Vec::<u32>::load_from(&query_path.join("perf_stats_num_buckets"))?;
        let s_num_queries = Vec::<u32>::load_from(&query_path.join("perf_stats_num_queries"))?;
        let s_cooperative = Vec::<u32>::load_from(&query_path.join("perf_stats_cooperative"))?;
        let s_sum_distance = Vec::<u64>::load_from(&query_path.join("perf_stats_sum_distance"))?;
        let s_avg_distance = Vec::<u64>::load_from(&query_path.join("perf_stats_avg_distance"))?;

        println!("Experiment has already been conducted, see results below!");
        for i in 0..s_sum_distance.len() {
            println!("--------------------------------------");
            println!(
                "Statistics for {} buckets (cooperative {}) after {} queries:",
                s_num_buckets[i],
                s_cooperative[i] > 0,
                s_num_queries[i]
            );
            println!("Distance sum: {} (avg: {})", s_sum_distance[i], s_avg_distance[i]);
        }
        println!("--------------------------------------");
        exit(0);
    }

    // init queries
    let mut queries = load_queries(&query_path)?;
    assert!(
        queries.len() as u32 >= *query_breakpoints.last().unwrap(),
        "Not enough queries in query set! Required {}, found {}",
        query_breakpoints.last().unwrap(),
        queries.len()
    );
    // bring queries into disorder -> required to enable faster traffic distribution
    permutate_queries(&mut queries);

    // init cch potential, use lowerbound graph
    let first_out = Vec::<u32>::load_from(&graph_path.join("first_out"))?;
    let head = Vec::<u32>::load_from(&graph_path.join("head"))?;
    let weight = Vec::<u32>::load_from(&graph_path.join("travel_time"))?;
    let weight = weight.iter().map(|&val| val * 1000).collect::<Vec<u32>>();

    let lower_bound = OwnedGraph::new(first_out, head, weight);

    let order = load_node_order(&graph_path)?;
    let cch = CCH::fix_order_and_build(&lower_bound, order);
    let cch_pot_data = CCHPotData::new(&cch, &lower_bound);

    // initialize graphs and store paths
    let graph_attributes = [(1, false)]
        .iter()
        .chain(graph_num_buckets.iter().map(|&v| (v, true)).collect::<Vec<(u32, bool)>>().iter())
        .cloned()
        .collect::<Vec<(u32, bool)>>();

    let mut path_results = vec![Vec::<PathResult>::with_capacity(queries.len()); graph_attributes.len()];

    let mut servers = graph_attributes
        .iter()
        .map(|&(num_buckets, _)| {
            let graph = load_capacity_graph(graph_path, num_buckets, bpr_traffic_function).unwrap();
            CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential())
        })
        .collect::<Vec<CapacityServer<BorrowedCCHPot>>>();
    println!("Initialized all Capacity Servers, starting queries..");

    let mut perf_statistics = Vec::new();

    for i in query_breakpoints.windows(2) {
        // perform queries on all server, store paths
        servers
            .par_iter_mut()
            .zip(graph_attributes.par_iter())
            .zip(path_results.par_iter_mut())
            .for_each(|((server, &(num_buckets, updates)), paths)| {
                (i[0] as usize..i[1] as usize)
                    .into_iter()
                    .zip(queries[i[0] as usize..i[1] as usize].iter())
                    .for_each(|(idx, query)| {
                        if (idx + 1) % 10000 == 0 {
                            println!("{} buckets - finished {} of {} queries", num_buckets, idx + 1, queries.len());
                        }

                        if let Some(result) = server.query(*query, updates) {
                            paths.push(result.path);
                        }
                    });
            });

        println!("Validating results after {} queries...", i[1]);

        // validate that each server produced valid results at the same time (or failed all)
        path_results.windows(2).for_each(|a| debug_assert_eq!(a[0].len(), a[1].len()));

        // evaluate all routes on the last server (which has the most buckets)
        let evaluation_server = servers.last().unwrap();

        let results = graph_attributes
            .par_iter()
            .zip(path_results.par_iter())
            .map(|(&(num_buckets, updates), paths)| {
                let sum_dist = paths.iter().map(|path| evaluation_server.path_distance(path) as u64).sum::<u64>();
                (num_buckets, i[1], updates, sum_dist, sum_dist / paths.len() as u64)
            })
            .collect::<Vec<(u32, u32, bool, u64, u64)>>();

        perf_statistics.extend_from_slice(&results);

        for (num_buckets, num_queries, cooperative, sum_distance, avg_distance) in results {
            println!("--------------------------------------");
            println!(
                "Statistics for {} buckets (cooperative {}) after {} queries:",
                num_buckets, cooperative, num_queries
            );
            println!("Distance sum: {} (avg: {})", sum_distance, avg_distance);
        }
        println!("--------------------------------------");
    }

    perf_statistics
        .iter()
        .map(|x| x.0)
        .collect::<Vec<u32>>()
        .write_to(&query_path.join("perf_stats_num_buckets"))?;
    perf_statistics
        .iter()
        .map(|x| x.1)
        .collect::<Vec<u32>>()
        .write_to(&query_path.join("perf_stats_num_queries"))?;
    perf_statistics
        .iter()
        .map(|x| x.2 as u32)
        .collect::<Vec<u32>>()
        .write_to(&query_path.join("perf_stats_cooperative"))?;
    perf_statistics
        .iter()
        .map(|x| x.3)
        .collect::<Vec<u64>>()
        .write_to(&query_path.join("perf_stats_sum_distance"))?;
    perf_statistics
        .iter()
        .map(|x| x.4)
        .collect::<Vec<u64>>()
        .write_to(&query_path.join("perf_stats_avg_distance"))?;

    Ok(())
}

fn parse_args() -> Result<(String, String, Vec<u32>, Vec<u32>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let breakpoints: String = parse_arg_required(&mut args, "Query breakpoints")?;
    let graph_buckets = parse_arg_optional(&mut args, "50,200,600".to_string());

    let mut query_breakpoints = ["0"]
        .iter()
        .cloned()
        .chain(breakpoints.split(","))
        .filter_map(|val| u32::from_str(val).ok())
        .collect::<Vec<u32>>();
    let mut graph_bucket_counts = graph_buckets.split(",").filter_map(|val| u32::from_str(val).ok()).collect::<Vec<u32>>();

    assert!(!query_breakpoints.is_empty() && !graph_bucket_counts.is_empty());

    // sort and remove duplicates
    query_breakpoints.sort();
    query_breakpoints.dedup();
    graph_bucket_counts.sort();
    graph_bucket_counts.dedup();

    Ok((graph_directory, query_directory, query_breakpoints, graph_bucket_counts))
}
