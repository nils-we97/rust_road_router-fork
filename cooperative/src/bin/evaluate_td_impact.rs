use std::env;
use std::error::Error;

use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::EdgeId;
use rust_road_router::report::measure;
use std::fs::File;
use std::io::Write;
use std::ops::Add;
use std::path::Path;
use std::str::FromStr;
use std::time::{Duration, Instant};

/// Evaluates the impact of time-dependence on a given set of queries.
/// Comparison is provided by a more static setting with only 1 bucket per edge.
/// The evaluation itself takes place on the time-dependent graph, with several steps (default: 50/200/400 buckets)
///
/// Expected result: with a rising number of long-distance queries, the time-dependent graph
/// should outperform the static setting (in terms of travel times).
///
/// To accelerate the queries, we use a Multi-Metric potential with default parameters
///
/// Additional parameters: <path_to_graph> <path_to_queries> <query_breakpoints> <buckets = 1,50,200,400>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, query_breakpoints, bucket_counts) = parse_args()?;
    let path = Path::new(&graph_directory);

    // load queries
    // queries will explicitly not be permuted! Otherwise, the time-dependent impact might be neglected
    let query_path = path.join("queries").join(query_directory);
    let queries = load_queries(&query_path)?;

    // init servers
    let mut servers = bucket_counts
        .iter()
        .map(|&num_buckets| {
            let graph = load_capacity_graph(&path, num_buckets, BPRTrafficFunction::default()).unwrap();
            let order = load_node_order(&path).unwrap();
            let cch = CCH::fix_order_and_build(&graph, order);

            let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &balanced_interval_pattern(), 20);
            CapacityServer::new(graph, customized)
        })
        .collect::<Vec<CapacityServer<CustomizedMultiMetrics>>>();
    println!("Initialized all Capacity Servers, starting queries..");

    let mut statistics = Vec::new();
    let mut path_results = vec![Vec::<Vec<EdgeId>>::with_capacity(queries.len()); bucket_counts.len()];
    let mut query_starts = vec![Vec::<Timestamp>::with_capacity(queries.len()); bucket_counts.len()];
    let mut query_times = vec![Duration::ZERO; bucket_counts.len()];

    for i in query_breakpoints.windows(2) {
        // perform queries on all server, store paths
        servers
            .par_iter_mut()
            .zip(bucket_counts.par_iter())
            .zip(path_results.par_iter_mut())
            .zip(query_starts.par_iter_mut())
            .zip(query_times.par_iter_mut())
            .for_each(|((((server, &num_buckets), paths), query_starts), total_time)| {
                let mut time = Instant::now();
                let mut idx = i[0];
                let mut last_update = 0;

                while idx < i[1] {
                    if (idx + 1) % 10000 == 0 {
                        println!(
                            "{} buckets - finished {} of {} queries - last step took {}s",
                            num_buckets,
                            idx + 1,
                            query_breakpoints.last().unwrap(),
                            time.elapsed().as_secs_f64()
                        );
                        time = Instant::now();
                    }

                    // check if regular re-customization must be executed before query
                    if (idx + 1) % 50000 == 0 {
                        let (_, time) = measure(|| server.customize(&balanced_interval_pattern(), 20));
                        *total_time = total_time.add(time);
                        last_update = idx;
                    }

                    let (result, time) = measure(|| server.query(&queries[idx as usize], true));
                    *total_time = total_time.add(time);

                    // check if the potential requires updates
                    if !server.result_valid() || !server.update_valid() {
                        if last_update == idx {
                            // panic to avoid infinite loops
                            panic!("{} - failed twice in the same step!", num_buckets);
                        } else {
                            // re-customization of upper bounds
                            last_update = idx;
                            println!("-- {} - potential update after {} steps", num_buckets, idx + 1);
                            let (_, time) = measure(|| server.customize_upper_bound());
                            *total_time = total_time.add(time);
                        }
                    }

                    // even if the update step violated some bounds, the result might still be valid!
                    if server.result_valid() {
                        if let Some(result) = result {
                            paths.push(result.path.edge_path);
                            query_starts.push(queries[idx as usize].departure);
                        }
                        idx += 1;
                    }
                }
            });

        println!("Validating results after {} queries...", i[1]);

        // validate that each server produced valid results at the same time
        // if they do not, this simplified assertion will likely fail after a few runs
        path_results.windows(2).for_each(|a| debug_assert_eq!(a[0].len(), a[1].len()));

        // evaluate all routes on the last server (which has the most buckets)
        let evaluation_server = servers.last().unwrap();

        let results = bucket_counts
            .par_iter()
            .zip(path_results.par_iter())
            .zip(query_starts.par_iter())
            .zip(query_times.par_iter())
            .map(|(((&num_buckets, paths), query_starts), time)| {
                let sum_dist = paths
                    .iter()
                    .zip(query_starts.iter())
                    .map(|(path, query_start)| evaluation_server.path_distance(path, *query_start) as u64)
                    .sum::<u64>();

                EvaluateTDImpactStatisticEntry::new(num_buckets, i[1], sum_dist, sum_dist / paths.len() as u64, paths.len(), *time)
            })
            .collect::<Vec<EvaluateTDImpactStatisticEntry>>();

        statistics.extend_from_slice(&results);

        for entry in results {
            println!("--------------------------------------");
            println!("Statistics for {} buckets after {} queries:", entry.num_buckets, entry.num_queries);
            println!(
                "Distance sum: {} (avg: {}, {} runs)",
                entry.sum_distance, entry.avg_distance, entry.num_valid_paths
            );
        }
        println!("--------------------------------------");
    }

    write_results(&statistics, &query_path.join("evaluate_td_impact.csv"))
}

fn write_results(results: &Vec<EvaluateTDImpactStatisticEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "num_buckets,num_queries,sum_distance,avg_distance,num_valid_paths,total_time_s\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!(
            "{},{},{},{},{},{}\n",
            entry.num_buckets,
            entry.num_queries,
            entry.sum_distance,
            entry.avg_distance,
            entry.num_valid_paths,
            entry.time.as_secs_f64()
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
    let graph_buckets = parse_arg_optional(&mut args, "1,50,200,400".to_string());

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

#[derive(Debug, Clone)]
struct EvaluateTDImpactStatisticEntry {
    pub num_buckets: u32,
    pub num_queries: u32,
    pub sum_distance: u64,
    pub avg_distance: u64,
    pub num_valid_paths: usize,
    pub time: Duration,
}

impl EvaluateTDImpactStatisticEntry {
    pub fn new(num_buckets: u32, num_queries: u32, sum_distance: u64, avg_distance: u64, num_valid_paths: usize, time: Duration) -> Self {
        Self {
            num_buckets,
            num_queries,
            sum_distance,
            avg_distance,
            num_valid_paths,
            time,
        }
    }
}
