use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::complete_balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::Graph;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::str::FromStr;
use std::time::Instant;

/// Evaluates the memory consumption of a cooperative routing approach.
/// After a given number of queries, the current memory consumption is evaluated, before further processing occurs.
/// Queries are accelerated with a default Multi-Metric potential which is updated after 50000 queries each
///
/// Additional parameters: <path_to_graph> <path_to_queries> <query_breakpoints, comma-separated> <buckets = 50,200,600>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, query_breakpoints, graph_bucket_counts) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

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

    let interval_pattern = complete_balanced_interval_pattern();

    // initialize graphs and evaluate memory consumption
    let usage_statistics = graph_bucket_counts
        .par_iter()
        .flat_map(|&num_buckets| {
            let mut statistics = Vec::with_capacity(query_breakpoints.len() - 1);

            // init graph and cch
            let graph = load_capacity_graph(graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
            let order = load_node_order(graph_path).unwrap();
            let cch = CCH::fix_order_and_build(&graph, order);

            // run initial customization, init server
            let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &interval_pattern, 20);
            let mut server = CapacityServer::new(graph, customized);
            println!("{} buckets - starting queries!", num_buckets);

            let mut query_time = Instant::now();

            for i in query_breakpoints.windows(2) {
                for idx in i[0] as usize..i[1] as usize {
                    server.query(&queries[idx], true);

                    if (idx + 1) % 10000 == 0 {
                        println!(
                            "{} buckets - finished {} of {} queries - last step took {}s",
                            num_buckets,
                            idx + 1,
                            query_breakpoints.last().unwrap(),
                            query_time.elapsed().as_secs_f64()
                        );
                        query_time = Instant::now();
                    }

                    // check if the potential has to be updated
                    if (idx + 1) % 50000 == 0 {
                        // regular re-customization
                        server.customize(&interval_pattern, 20);
                    } else if !server.result_valid() || !server.update_valid() {
                        // re-customization of upper bounds
                        println!("-- {} - potential update after {} steps", num_buckets, idx + 1);
                        server.customize_upper_bound();
                    }
                }

                let (num_used_edges, num_used_buckets) = server.borrow_graph().get_bucket_usage();

                statistics.push(EvaluateCoopStorageStatisticEntry::new(
                    num_buckets,
                    i[1],
                    num_used_buckets as f64 / (server.borrow_graph().num_buckets() * server.borrow_graph().num_arcs() as u32) as f64,
                    num_used_buckets,
                    num_used_edges as f64 / server.borrow_graph().num_arcs() as f64,
                    num_used_edges,
                    server.borrow_graph().get_mem_size(),
                ));
            }
            statistics
        })
        .collect::<Vec<EvaluateCoopStorageStatisticEntry>>();

    write_results(&usage_statistics, &query_path.join("evaluate_cooperative_storage.csv"))
}

fn write_results(results: &Vec<EvaluateCoopStorageStatisticEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "num_buckets,num_queries,bucket_usage_rel,bucket_usage_abs,edge_usage_rel,edge_usage_abs,memory_usage\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!(
            "{},{},{},{},{},{},{}\n",
            entry.num_buckets,
            entry.num_queries,
            entry.bucket_usage_rel,
            entry.bucket_usage_abs,
            entry.edge_usage_rel,
            entry.edge_usage_abs,
            entry.memory_usage
        );
        file.write(line.as_bytes())?;
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

struct EvaluateCoopStorageStatisticEntry {
    pub num_buckets: u32,
    pub num_queries: u32,
    pub bucket_usage_rel: f64,
    pub bucket_usage_abs: usize,
    pub edge_usage_rel: f64,
    pub edge_usage_abs: usize,
    pub memory_usage: usize,
}

impl EvaluateCoopStorageStatisticEntry {
    pub fn new(
        num_buckets: u32,
        num_queries: u32,
        bucket_usage_rel: f64,
        bucket_usage_abs: usize,
        edge_usage_rel: f64,
        edge_usage_abs: usize,
        memory_usage: usize,
    ) -> Self {
        Self {
            num_buckets,
            num_queries,
            bucket_usage_rel,
            bucket_usage_abs,
            edge_usage_rel,
            edge_usage_abs,
            memory_usage,
        }
    }
}
