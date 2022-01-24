use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::complete_balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
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
use std::path::Path;
use std::str::FromStr;
use std::time::{Duration, Instant};

/// Evaluates the quality of A*-Potentials with a rising number of queries.
/// As the potential metrics are not updated in every step, its quality will possibly
/// get worse with with every additional query.
///
/// Compares the static lowerbound A* approach with the Multi-Metrics potential
/// Updates will occur whenever needed as well as each 100.000 queries by default
///
/// Additional parameters: <path_to_graph> <path_to_queries> <num_buckets> <num_pot_metrics> <query evaluation breakpoints, comma-separated> <cust-breakpoints = 100.000>
/// Note that all customization breakpoints must be divisible by all evaluation breakpoints
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, num_buckets, mm_num_metrics, evaluation_breakpoints, customization_breakpoints) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // verify correctness of customization and evaluation breakpoints
    evaluation_breakpoints[1..].iter().for_each(|&a| {
        customization_breakpoints.iter().for_each(|&b| {
            assert_eq!(
                a % b,
                0,
                "Cust-Breakpoints ({:?}) must be divisible by all Eval-Breakpoints ({:?})",
                customization_breakpoints,
                evaluation_breakpoints
            );
        })
    });

    // init queries
    let mut queries = load_queries(&query_path)?;
    assert!(
        queries.len() as u32 >= *evaluation_breakpoints.last().unwrap(),
        "Not enough queries in query set! Required {}, found {}",
        evaluation_breakpoints.last().unwrap(),
        queries.len()
    );
    // bring queries into disorder -> required to enable faster traffic distribution
    permutate_queries(&mut queries);

    // load cch
    let temp_graph = load_capacity_graph(graph_path, num_buckets, BPRTrafficFunction::default())?;

    let interval_pattern = complete_balanced_interval_pattern();

    let results = [(PotentialType::CCHPot, *customization_breakpoints.last().unwrap())]
        .par_iter()
        .cloned()
        .chain(customization_breakpoints.par_iter().map(|&p| (PotentialType::MultiMetrics, p)))
        .chain(customization_breakpoints.par_iter().map(|&p| (PotentialType::CorridorLowerbound, p)))
        .flat_map(|(potential_type, update_frequency)| {
            let heuristic_name = format!("{}-{}", update_frequency, potential_type.to_string());

            // load graph
            let graph = load_capacity_graph(&graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
            println!("{}: Graph initialized!", heuristic_name);

            // init cch
            let order = load_node_order(graph_path).unwrap();
            let (cch, time) = measure(|| CCH::fix_order_and_build(&temp_graph, order));
            println!("{}: CCH created in {} ms", heuristic_name, time.as_secs_f64() * 1000.0);

            let mut total_time_query = Duration::ZERO;
            let mut total_time_potential = Duration::ZERO;
            let mut total_time_update = Duration::ZERO;
            let mut total_time_reinit = Duration::ZERO;

            let mut time_query = Duration::ZERO;
            let mut time_potential = Duration::ZERO;
            let mut time_update = Duration::ZERO;
            let mut sum_dist = 0;
            let mut num_runs = 0;

            let mut statistics = Vec::new();

            match potential_type {
                PotentialType::CCHPot => {
                    let init_start = Instant::now();
                    let cch_pot_data = CCHPotData::new(&cch, &graph);
                    let mut server = CapacityServer::new(graph, cch_pot_data.forward_potential());
                    total_time_reinit = total_time_reinit.add(init_start.elapsed());

                    for a in evaluation_breakpoints.windows(2) {
                        for current_idx in a[0]..a[1] {
                            execute_query(
                                &mut server,
                                heuristic_name.as_str(),
                                &queries[current_idx as usize],
                                current_idx as usize,
                                &mut time_potential,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );
                            // no re-customization required!
                        }

                        statistics.push(EvaluatePotentialQualityStatisticEntry::new(
                            heuristic_name.clone(),
                            a[1],
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));
                    }
                }
                PotentialType::CorridorLowerbound => {
                    let mut last_update_step = 0;
                    // init server
                    let init_start = Instant::now();
                    let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &graph, 72);
                    let mut server = CapacityServer::new(graph, customized);
                    total_time_reinit = total_time_reinit.add(init_start.elapsed());

                    // execute all queries
                    for a in evaluation_breakpoints.windows(2) {
                        let mut current_idx = a[0];

                        while current_idx < a[1] {
                            // check if regular re-customization must be executed before query
                            if (current_idx + 1) % update_frequency == 0 {
                                let (_, time) = measure(|| {
                                    let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &server.borrow_graph(), 72);
                                    server.customize(customized);
                                });
                                total_time_reinit = total_time_reinit.add(time);
                                last_update_step = current_idx;
                            }

                            execute_query(
                                &mut server,
                                heuristic_name.as_str(),
                                &queries[current_idx as usize],
                                current_idx as usize,
                                &mut time_potential,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            // check if the potential requires update
                            if !server.result_valid() || !server.update_valid() {
                                // avoid infinity loops - panic if the bounds are not updated properly
                                if last_update_step == current_idx {
                                    panic!("Failed twice in the same step! Query: {:?}", &queries[current_idx as usize]);
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
                            current_idx += server.result_valid() as u32;
                        }

                        statistics.push(EvaluatePotentialQualityStatisticEntry::new(
                            heuristic_name.clone(),
                            a[1],
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));
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
                    for a in evaluation_breakpoints.windows(2) {
                        let mut current_idx = a[0];

                        while current_idx < a[1] {
                            // check if regular re-customization must be executed before query
                            if (current_idx + 1) % update_frequency == 0 {
                                let (_, time) = measure(|| server.customize(&interval_pattern, mm_num_metrics as usize));
                                total_time_reinit = total_time_reinit.add(time);
                                last_update_step = current_idx;
                            }

                            execute_query(
                                &mut server,
                                heuristic_name.as_str(),
                                &queries[current_idx as usize],
                                current_idx as usize,
                                &mut time_potential,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut num_runs,
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            // check if the potential requires update
                            if !server.result_valid() || !server.update_valid() {
                                // avoid infinity loops - panic if the bounds are not updated properly
                                if last_update_step == current_idx {
                                    panic!("Failed twice in the same step! Query: {:?}", &queries[current_idx as usize]);
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
                            current_idx += server.result_valid() as u32;
                        }

                        statistics.push(EvaluatePotentialQualityStatisticEntry::new(
                            heuristic_name.clone(),
                            a[1],
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));
                    }
                }
                PotentialType::MultiLevelBucket => {}
            }

            statistics
        })
        .collect::<Vec<EvaluatePotentialQualityStatisticEntry>>();

    // output data and write csv
    write_results(&results, &query_path.join("evaluate_pot_quality.csv"))
}

fn execute_query<Server: CapacityServerOps>(
    server: &mut Server,
    name: &str,
    query: &TDQuery<Timestamp>,
    idx: usize,
    time_potential: &mut Duration,
    time_query: &mut Duration,
    time_update: &mut Duration,
    sum_dist: &mut u64,
    num_runs: &mut u64,
    total_time_potential: &mut Duration,
    total_time_query: &mut Duration,
    total_time_update: &mut Duration,
) {
    let query_result = server.query_measured(query, true);
    *time_potential = time_potential.add(query_result.distance_result.time_potential);
    *time_query = time_query.add(query_result.distance_result.time_query);
    *time_update = time_update.add(query_result.update_time);

    if let Some(distance) = query_result.query_result.map(|r| r.distance) {
        *sum_dist += distance as u64;
        *num_runs += 1;
    }

    *total_time_query = total_time_query.add(query_result.distance_result.time_query);
    *total_time_potential = total_time_potential.add(query_result.distance_result.time_potential);
    *total_time_update = total_time_update.add(query_result.update_time);

    /*println!(
        "{} - query result: {:?}, potential: {:?}",
        name, &query_result.distance_result.distance, &query_result.distance_result.potential
    );*/
    if (idx + 1) % 1000 == 0 {
        println!(
            "{}: Finished {} queries. Last step: {}s pot init, {}s query, {}s ttf update - avg dist: {}, {} valid runs",
            name,
            idx + 1,
            time_potential.as_secs_f64(),
            time_query.as_secs_f64(),
            time_update.as_secs_f64(),
            *sum_dist / max(*num_runs, 1),
            *num_runs
        );

        *time_potential = Duration::ZERO;
        *time_query = Duration::ZERO;
        *time_update = Duration::ZERO;
        *sum_dist = 0;
        *num_runs = 0;
    }
}

fn write_results(results: &Vec<EvaluatePotentialQualityStatisticEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "name,query_count,time_potential,time_query,time_update,time_reinit\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!(
            "{},{},{},{},{},{}\n",
            entry.pot_name,
            entry.num_queries,
            entry.time_potential.as_secs_f64() * 1000.0,
            entry.time_query.as_secs_f64() * 1000.0,
            entry.time_update.as_secs_f64() * 1000.0,
            entry.time_reinit.as_secs_f64() * 1000.0
        );
        file.write(line.as_bytes())?;

        println!("------------------------------------");
        println!("Runtime Statistics for {} after {} queries", entry.pot_name, entry.num_queries);
        println!(
            "Potential init: {}, query: {}, update: {}, reinit: {}",
            entry.time_potential.as_secs_f64(),
            entry.time_query.as_secs_f64(),
            entry.time_update.as_secs_f64(),
            entry.time_reinit.as_secs_f64()
        );
    }

    Ok(())
}

fn parse_args() -> Result<(String, String, u32, u32, Vec<u32>, Vec<u32>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let num_buckets = parse_arg_required(&mut args, "number of buckets")?;
    let cl_num_intervals = parse_arg_required(&mut args, "number of intervals [corridor-lowerbound]")?;
    let evaluation_breakpoints: String = parse_arg_required(&mut args, "Query breakpoints")?;
    let customization_breakpoints = parse_arg_optional(&mut args, "1".to_string());

    let mut evaluation_breakpoints = ["0"]
        .iter()
        .cloned()
        .chain(evaluation_breakpoints.split(","))
        .filter_map(|val| u32::from_str(val).ok())
        .collect::<Vec<u32>>();

    let mut customization_breakpoints = customization_breakpoints
        .split(",")
        .filter_map(|val| u32::from_str(val).ok())
        .collect::<Vec<u32>>();

    assert!(!evaluation_breakpoints.is_empty() && !customization_breakpoints.is_empty());

    // sort and remove duplicates
    evaluation_breakpoints.sort();
    evaluation_breakpoints.dedup();
    customization_breakpoints.sort();
    customization_breakpoints.dedup();

    Ok((
        graph_directory,
        query_directory,
        num_buckets,
        cl_num_intervals,
        evaluation_breakpoints,
        customization_breakpoints,
    ))
}

struct EvaluatePotentialQualityStatisticEntry {
    pub pot_name: String,
    pub num_queries: u32,
    pub time_potential: Duration,
    pub time_query: Duration,
    pub time_update: Duration,
    pub time_reinit: Duration,
}

impl EvaluatePotentialQualityStatisticEntry {
    pub fn new(pot_name: String, num_queries: u32, time_potential: Duration, time_query: Duration, time_update: Duration, time_reinit: Duration) -> Self {
        Self {
            pot_name,
            num_queries,
            time_potential,
            time_query,
            time_update,
            time_reinit,
        }
    }
}
