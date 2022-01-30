use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::complete_balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::types::PotentialType;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::report::measure;
use std::cmp::max;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::ops::Add;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Evaluates the quality of A*-Potentials with a rising number of queries.
/// As the potential metrics are not updated in every step, its quality will possibly
/// get worse with with every additional query.
///
/// In this setting, the query sets are ordered by timestamp, and executed independently from each other.
///
/// Potential updates will occur whenever needed as well as each x queries
///
/// Additional parameters: <path_to_graph> <num_buckets> <path_to_queries, comma-separated> <num_mm_pot_metrics = 20> <mm_update_frequency = 50000> <num_cl_pot_intervals = 72> <cl_update_frequency = 100000>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, num_buckets, query_directories, mm_num_metrics, mm_update_frequency, cl_num_intervals, cl_update_frequency) = parse_args()?;

    let graph_path = Path::new(&graph_directory);

    // validate query paths
    let query_paths = query_directories
        .iter()
        .map(|v| {
            let path = graph_path.join("queries").join(v);
            assert!(path.exists());
            path
        })
        .collect::<Vec<PathBuf>>();

    let order = load_node_order(&graph_path)?;
    let interval_pattern = complete_balanced_interval_pattern();
    let mut result = Vec::new();

    // resolve query sets one after another
    for query_path in query_paths {
        let queries = load_queries(&query_path)?;

        // initialize servers, run queries
        let current_results = vec![PotentialType::CCHPot, PotentialType::MultiMetrics, PotentialType::CorridorLowerbound]
            .par_iter()
            .flat_map(|pot_type| {
                let name = format!("{}-{}", pot_type.to_string(), queries.len());

                // load graph
                let graph = load_capacity_graph(&graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
                println!("{}: Graph initialized!", &name);

                // init cch
                let (cch, time) = measure(|| CCH::fix_order_and_build(&graph, order.clone()));
                println!("{}: CCH created in {} ms", &name, time.as_secs_f64() * 1000.0);

                let mut total_time_query = Duration::ZERO;
                let mut total_time_update = Duration::ZERO;
                let mut total_time_reinit = Duration::ZERO;

                let mut time_query = Duration::ZERO;
                let mut time_update = Duration::ZERO;
                let mut sum_dist = 0;
                let mut num_runs = 0;

                match pot_type {
                    PotentialType::CCHPot => {
                        let init_start = Instant::now();
                        let cch_pot_data = CCHPotData::new(&cch, &graph);
                        let mut server = CapacityServer::new(graph, cch_pot_data.forward_potential());
                        total_time_reinit = total_time_reinit.add(init_start.elapsed());

                        queries.iter().enumerate().for_each(|(idx, query)| {
                            execute_query(
                                &mut server,
                                name.as_str(),
                                query,
                                idx,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_query,
                                &mut total_time_update,
                            );
                        });
                    }
                    PotentialType::CorridorLowerbound => {
                        let mut last_update_step = 0;
                        // init server
                        let init_start = Instant::now();
                        let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &graph, cl_num_intervals);
                        let mut server = CapacityServer::new(graph, customized);
                        total_time_reinit = total_time_reinit.add(init_start.elapsed());

                        // execute all queries
                        let mut current_idx = 0;
                        while current_idx < queries.len() {
                            // check if regular re-customization must be executed before query
                            if (current_idx as u32 + 1) % cl_update_frequency == 0 && current_idx + 1 < queries.len() {
                                let (_, time) = measure(|| {
                                    let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &server.borrow_graph(), 72);
                                    server.customize(customized);
                                });
                                total_time_reinit = total_time_reinit.add(time);
                                last_update_step = current_idx;
                            }

                            execute_query(
                                &mut server,
                                name.as_str(),
                                &queries[current_idx],
                                current_idx,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            // check if the potential requires update
                            if !server.result_valid() || !server.update_valid() {
                                // avoid infinity loops - panic if the bounds are not updated properly
                                if last_update_step == current_idx {
                                    panic!("Failed twice in the same step! Query: {:?}", &queries[current_idx]);
                                } else {
                                    last_update_step = current_idx;

                                    println!("\n\n--------------------------");
                                    println!("Corridor-Lowerbound: Update Bounds in step {}", current_idx);
                                    println!("--------------------------\n\n");

                                    let (_, time) = measure(|| server.customize_upper_bound(&cch));
                                    total_time_reinit = total_time_reinit.add(time);
                                }
                            }

                            // even if the update step violated some bounds, the result might still be valid
                            current_idx += server.result_valid() as usize;
                        }
                    }
                    PotentialType::MultiMetrics => {
                        let mut last_update_step = 0;
                        // init server
                        let init_start = Instant::now();
                        let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &interval_pattern, mm_num_metrics as usize);
                        let mut server = CapacityServer::new(graph, customized);
                        total_time_reinit = total_time_reinit.add(init_start.elapsed());

                        // execute all queries
                        let mut current_idx = 0;
                        while current_idx < queries.len() {
                            // check if regular re-customization must be executed before query
                            if (current_idx as u32 + 1) % mm_update_frequency == 0 && current_idx + 1 < queries.len() {
                                let (_, time) = measure(|| server.customize(&interval_pattern, mm_num_metrics as usize));
                                total_time_reinit = total_time_reinit.add(time);
                                last_update_step = current_idx;
                            }

                            execute_query(
                                &mut server,
                                name.as_str(),
                                &queries[current_idx],
                                current_idx,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            // check if the potential requires update
                            if !server.result_valid() || !server.update_valid() {
                                // avoid infinity loops - panic if the bounds are not updated properly
                                if last_update_step == current_idx {
                                    panic!("Failed twice in the same step! Query: {:?}", &queries[current_idx]);
                                } else {
                                    last_update_step = current_idx;

                                    println!("\n\n--------------------------");
                                    println!("Multi-Metric: Update Bounds in step {}", current_idx);
                                    println!("--------------------------\n\n");

                                    let (_, time) = measure(|| server.customize_upper_bound());
                                    total_time_reinit = total_time_reinit.add(time);
                                }
                            }

                            // even if the update step violated some bounds, the result might still be valid
                            current_idx += server.result_valid() as usize;
                        }
                    }
                }

                // push an entry for all different measures
                [("reinit", total_time_reinit), ("update", total_time_update), ("query", total_time_query)]
                    .iter()
                    .map(|&(time_type, time)| EvaluatePotQualityResultEntry::new(pot_type.to_string(), queries.len() as u32, time_type.to_string(), time))
                    .collect::<Vec<EvaluatePotQualityResultEntry>>()
            })
            .collect::<Vec<EvaluatePotQualityResultEntry>>();

        result.extend_from_slice(&current_results);
    }

    write_results(&result, &graph_path.join("evaluate_potential_quality_ordered.csv"))
}

fn execute_query<Server: CapacityServerOps>(
    server: &mut Server,
    name: &str,
    query: &TDQuery<Timestamp>,
    idx: usize,
    time_query: &mut Duration,
    time_update: &mut Duration,
    sum_dist: &mut u64,
    num_runs: &mut u64,
    total_time_query: &mut Duration,
    total_time_update: &mut Duration,
) {
    let query_result = server.query_measured(query, true);
    *time_query = time_query
        .add(query_result.distance_result.time_query)
        .add(query_result.distance_result.time_potential);
    *time_update = time_update.add(query_result.update_time);

    if let Some(distance) = query_result.query_result.map(|r| r.distance) {
        *sum_dist += distance as u64;
        *num_runs += 1;
    }

    *total_time_query = total_time_query
        .add(query_result.distance_result.time_query)
        .add(query_result.distance_result.time_potential);
    *total_time_update = total_time_update.add(query_result.update_time);

    if (idx + 1) % 1000 == 0 {
        println!(
            "{}: Finished {} queries. Last step: {}s query, {}s ttf update - avg dist: {}, {} valid runs",
            name,
            idx + 1,
            time_query.as_secs_f64(),
            time_update.as_secs_f64(),
            *sum_dist / max(*num_runs, 1),
            *num_runs
        );

        *time_query = Duration::ZERO;
        *time_update = Duration::ZERO;
        *sum_dist = 0;
        *num_runs = 0;
    }
}

fn write_results(results: &Vec<EvaluatePotQualityResultEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "name,query_count,time_type,time\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!("{},{},{},{}\n", entry.name, entry.query_count, entry.time_type, entry.time.as_secs_f64());
        file.write(line.as_bytes())?;
    }

    Ok(())
}

fn parse_args() -> Result<(String, u32, Vec<String>, u32, u32, u32, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets = parse_arg_required(&mut args, "number of buckets")?;
    let query_directories: String = parse_arg_required(&mut args, "Query Directory")?;
    let mm_num_metrics = parse_arg_optional(&mut args, 20);
    let mm_update_frequency = parse_arg_optional(&mut args, 50000);
    let cl_num_intervals = parse_arg_optional(&mut args, 72);
    let cl_update_frequency = parse_arg_optional(&mut args, 100000);

    let query_directories = query_directories.split(",").map(|s| s.to_string()).collect::<Vec<String>>();

    assert!(!query_directories.is_empty());

    Ok((
        graph_directory,
        num_buckets,
        query_directories,
        mm_num_metrics,
        mm_update_frequency,
        cl_num_intervals,
        cl_update_frequency,
    ))
}

#[derive(Clone, Debug)]
struct EvaluatePotQualityResultEntry {
    pub name: String,
    pub query_count: u32,
    pub time_type: String,
    pub time: Duration,
}

impl EvaluatePotQualityResultEntry {
    pub fn new(name: String, query_count: u32, time_type: String, time: Duration) -> Self {
        Self {
            name,
            query_count,
            time_type,
            time,
        }
    }
}
