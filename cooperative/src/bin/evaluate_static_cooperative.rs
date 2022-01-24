use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::EdgeId;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::str::FromStr;
use std::time::Instant;

/// Evaluate the difference between static (no updates) and cooperative routing.
/// In the static setting, no updates take place after calculating the routes.
/// The evaluation takes place on the frequently updated graph.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// In order to accelerate the queries, a Multi-Metric potential with default parameters is used
///
/// Additional parameters: <path_to_graph> <path_to_queries> <query_breakpoints, comma-separated> <query_buckets=50,200,600>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, query_breakpoints, graph_num_buckets) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // init queries, do NOT permute them
    let mut queries = load_queries(&query_path)?;
    assert!(
        queries.len() as u32 >= *query_breakpoints.last().unwrap(),
        "Not enough queries in query set! Required {}, found {}",
        query_breakpoints.last().unwrap(),
        queries.len()
    );

    // initialize graphs and store paths
    let graph_attributes = [(1, false)]
        .iter()
        .chain(graph_num_buckets.iter().map(|&v| (v, true)).collect::<Vec<(u32, bool)>>().iter())
        .cloned()
        .collect::<Vec<(u32, bool)>>();

    let mut path_results = vec![Vec::<Vec<EdgeId>>::with_capacity(queries.len()); graph_attributes.len()];
    let mut query_starts = vec![Vec::<Timestamp>::with_capacity(queries.len()); graph_attributes.len()];

    let mut servers = graph_attributes
        .iter()
        .map(|&(num_buckets, _)| {
            let graph = load_capacity_graph(graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
            let order = load_node_order(&graph_path).unwrap();
            let cch = CCH::fix_order_and_build(&graph, order);

            let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &balanced_interval_pattern(), 20);
            CapacityServer::new(graph, customized)
        })
        .collect::<Vec<CapacityServer<CustomizedMultiMetrics>>>();
    println!("Initialized all Capacity Servers, starting queries..");

    let mut perf_statistics = Vec::new();

    for i in query_breakpoints.windows(2) {
        // perform queries on all server, store paths
        servers
            .par_iter_mut()
            .zip(graph_attributes.par_iter())
            .zip(path_results.par_iter_mut())
            .zip(query_starts.par_iter_mut())
            .for_each(|(((server, &(num_buckets, updates)), paths), query_starts)| {
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
                        server.customize(&balanced_interval_pattern(), 20);
                        last_update = idx;
                    }

                    let result = server.query(&queries[idx as usize], updates);

                    // check if the potential requires updates
                    if !server.result_valid() || !server.update_valid() {
                        if last_update == idx {
                            // panic to avoid infinite loops
                            panic!("{} - failed twice in the same step!", num_buckets);
                        } else {
                            // re-customization of upper bounds
                            last_update = idx;
                            println!("-- {} - potential update after {} steps", num_buckets, idx + 1);
                            server.customize_upper_bound();
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

        // validate that each server produced valid results at the same time (or failed all)
        path_results.windows(2).for_each(|a| debug_assert_eq!(a[0].len(), a[1].len()));

        // evaluate all routes on the last server (which has the most buckets)
        let evaluation_server = servers.last().unwrap();

        let results = graph_attributes
            .par_iter()
            .zip(path_results.par_iter())
            .zip(query_starts.par_iter())
            .map(|((&(num_buckets, updates), paths), query_starts)| {
                let sum_dist = paths
                    .iter()
                    .zip(query_starts.iter())
                    .map(|(path, query_start)| evaluation_server.path_distance(path, *query_start) as u64)
                    .sum::<u64>();

                EvaluateStaticCooperativeStatisticEntry::new(num_buckets, i[1], updates, sum_dist, sum_dist / paths.len() as u64)
            })
            .collect::<Vec<EvaluateStaticCooperativeStatisticEntry>>();

        perf_statistics.extend_from_slice(&results);

        for entry in results {
            println!("--------------------------------------");
            println!(
                "Statistics for {} buckets (cooperative {}) after {} queries:",
                entry.num_buckets, entry.cooperative, entry.num_queries
            );
            println!("Distance sum: {} (avg: {})", entry.sum_distance, entry.avg_distance);
        }
        println!("--------------------------------------");
    }

    write_results(&perf_statistics, &query_path.join("evaluate_static_cooperative.csv"))
}

fn write_results(results: &Vec<EvaluateStaticCooperativeStatisticEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "num_buckets,num_queries,cooperative,sum_distance,avg_distance\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!(
            "{},{},{},{},{}\n",
            entry.num_buckets, entry.num_queries, entry.cooperative as u32, entry.sum_distance, entry.avg_distance
        );
        file.write(line.as_bytes())?;
    }

    Ok(())
}

fn parse_args() -> Result<(String, String, Vec<u32>, Vec<u32>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let breakpoints: String = parse_arg_required(&mut args, "Query breakpoints")?;
    let graph_buckets = parse_arg_optional(&mut args, "50,200,400".to_string());

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

#[derive(Clone, Debug)]
struct EvaluateStaticCooperativeStatisticEntry {
    pub num_buckets: u32,
    pub num_queries: u32,
    pub cooperative: bool,
    pub sum_distance: u64,
    pub avg_distance: u64,
}

impl EvaluateStaticCooperativeStatisticEntry {
    pub fn new(num_buckets: u32, num_queries: u32, cooperative: bool, sum_distance: u64, avg_distance: u64) -> Self {
        Self {
            num_buckets,
            num_queries,
            cooperative,
            sum_distance,
            avg_distance,
        }
    }
}
