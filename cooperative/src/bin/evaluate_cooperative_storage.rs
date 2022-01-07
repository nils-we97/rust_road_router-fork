use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::{Graph, OwnedGraph};
use rust_road_router::io::{Load, Store};
use std::env;
use std::error::Error;
use std::path::Path;
use std::process::exit;
use std::str::FromStr;

/// Evaluates the memory consumption of a cooperative routing approach.
/// After a given number of queries, the current memory consumption is evaluated, before further processing occurs.
///
/// Additional parameters: <path_to_graph> <path_to_queries> <query_breakpoints, comma-separated> <buckets = 50,200,600>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, query_breakpoints, graph_bucket_counts) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // read statistic results
    if query_path.join("perf_stats_num_buckets").exists() {
        let s_num_buckets = Vec::<u32>::load_from(&query_path.join("stats_num_buckets"))?;
        let s_num_queries = Vec::<u32>::load_from(&query_path.join("stats_num_queries"))?;
        let s_bucket_usage = Vec::<f64>::load_from(&query_path.join("stats_bucket_usage"))?;
        let s_bucket_usage_abs = Vec::<u64>::load_from(&query_path.join("stats_bucket_usage_abs"))?;
        let s_memory_usage = Vec::<u64>::load_from(&query_path.join("stats_memory_usage"))?;

        println!("Experiment has already been conducted, see results below!");
        for i in 0..s_num_buckets.len() {
            println!("---------------------------------");
            println!("Statistics for {} buckets, after {} queries:", s_num_buckets[i], s_num_queries[i]);
            println!(
                "Memory consumption: {} bytes, bucket usage: {}% (absolute: {})",
                s_memory_usage[i],
                s_bucket_usage[i] * 100.0,
                s_bucket_usage_abs[i]
            );
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

    // initialize graphs and evaluate memory consumption
    let usage_statistics = graph_bucket_counts
        .par_iter()
        .flat_map(|&num_buckets| {
            let mut statistics = Vec::with_capacity(query_breakpoints.len() - 1);

            let graph = load_capacity_graph(graph_path, num_buckets, bpr_traffic_function).unwrap();
            let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
            println!("{} buckets - starting queries!", num_buckets);

            for i in query_breakpoints.windows(2) {
                (i[0] as usize..i[1] as usize)
                    .into_iter()
                    .zip(queries[i[0] as usize..i[1] as usize].iter())
                    .for_each(|(idx, query)| {
                        if (idx + 1) % 10000 == 0 {
                            println!("{} buckets - finished {} of {} queries", num_buckets, idx + 1, queries.len());
                        }

                        server.query(*query, true);
                    });

                let bucket_usage =
                    server.borrow_graph().get_bucket_usage() as f64 / (server.borrow_graph().num_buckets() * server.borrow_graph().num_arcs() as u32) as f64;
                let memory_usage = server.borrow_graph().get_mem_size();

                statistics.push((num_buckets, i[1], bucket_usage, server.borrow_graph().get_bucket_usage(), memory_usage));
            }
            statistics
        })
        .collect::<Vec<(u32, u32, f64, usize, usize)>>();

    usage_statistics
        .iter()
        .map(|x| x.0)
        .collect::<Vec<u32>>()
        .write_to(&query_path.join("stats_num_buckets"))?;
    usage_statistics
        .iter()
        .map(|x| x.1)
        .collect::<Vec<u32>>()
        .write_to(&query_path.join("stats_num_queries"))?;
    usage_statistics
        .iter()
        .map(|x| x.2)
        .collect::<Vec<f64>>()
        .write_to(&query_path.join("stats_bucket_usage"))?;
    usage_statistics
        .iter()
        .map(|x| x.3 as u64)
        .collect::<Vec<u64>>()
        .write_to(&query_path.join("stats_bucket_usage_abs"))?;
    usage_statistics
        .iter()
        .map(|x| x.4 as u64)
        .collect::<Vec<u64>>()
        .write_to(&query_path.join("stats_memory_usage"))?;

    for (num_buckets, num_queries, bucket_usage, bucket_usage_abs, memory_usage) in usage_statistics {
        println!("---------------------------------");
        println!("Statistics for {} buckets, after {} queries:", num_buckets, num_queries);
        println!(
            "Memory consumption: {} bytes, bucket usage: {}% (absolute: {})",
            memory_usage,
            bucket_usage * 100.0,
            bucket_usage_abs
        );
    }

    Ok(())
}

fn parse_args() -> Result<(String, String, Vec<u32>, Vec<u32>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
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
