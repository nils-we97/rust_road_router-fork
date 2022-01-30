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
use std::time::{Duration, Instant};

/// Evaluates the quality of A*-Potentials with a rising number of queries.
/// As the potential metrics are not updated in every step, its quality will possibly
/// get worse with with every additional query.
///
/// In this setting, the query set is unordered by departure time, and the results are captured frequently.
///
/// Potential updates will occur whenever needed as well as each x queries
///
/// Additional parameters: <path_to_graph> <path_to_queries> <num_buckets> <query_evaluation_frequency = 100000> <mm_num_metrics = 20> <mm_update_frequency = 50000> <cl_num_intervals = 72> <cl_update_frequency = 72>
/// Note that `query_evaluation_frequency` must be divisible by the total number of queries
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, num_buckets, evaluation_frequency, mm_num_metrics, mm_update_frequency, cl_num_intervals, cl_update_frequency) =
        parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // init queries, bring them into disorder to enable faster traffic distribution over the day
    let mut queries = load_queries(&query_path)?;
    permutate_queries(&mut queries);

    // resolve evaluation breakpoints
    assert_eq!(
        queries.len() as u32 % evaluation_frequency,
        0,
        "Number of queries ({}) must be divisible by the evaluation frequency ({})",
        queries.len(),
        evaluation_frequency
    );
    let evaluation_breakpoints = (0..=(queries.len() as u32 / evaluation_frequency))
        .map(|i| i * evaluation_frequency)
        .collect::<Vec<u32>>();

    let order = load_node_order(&graph_path)?;
    let interval_pattern = complete_balanced_interval_pattern();

    let results = [PotentialType::CCHPot, PotentialType::MultiMetrics, PotentialType::CorridorLowerbound]
        .par_iter()
        .flat_map(|potential_type| {
            // load graph
            let graph = load_capacity_graph(&graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
            println!("{}: Graph initialized!", potential_type.to_string());

            // init cch
            let (cch, time) = measure(|| CCH::fix_order_and_build(&graph, order.clone()));
            println!("{}: CCH created in {} ms", potential_type.to_string(), time.as_secs_f64() * 1000.0);

            let mut total_time = Duration::ZERO;
            let mut temp_time = Duration::ZERO;
            let mut sum_dist = 0;
            let mut num_runs = 0;

            match potential_type {
                PotentialType::CCHPot => {
                    let init_start = Instant::now();
                    let cch_pot_data = CCHPotData::new(&cch, &graph);
                    let mut server = CapacityServer::new(graph, cch_pot_data.forward_potential());
                    total_time = total_time.add(init_start.elapsed());

                    evaluation_breakpoints
                        .windows(2)
                        .map(|a| {
                            (a[0] as usize..a[1] as usize).into_iter().for_each(|current_idx| {
                                execute_query(
                                    &mut server,
                                    &potential_type.to_string(),
                                    &queries[current_idx],
                                    current_idx,
                                    &mut temp_time,
                                    &mut sum_dist,
                                    &mut num_runs,
                                    &mut total_time,
                                );
                                // no re-customization required!
                            });

                            EvaluatePotQualityEntry::new(potential_type.to_string(), a[1], total_time)
                        })
                        .collect::<Vec<EvaluatePotQualityEntry>>()
                }
                PotentialType::CorridorLowerbound => {
                    let mut last_update_step = 0;
                    // init server
                    let init_start = Instant::now();
                    let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &graph, cl_num_intervals);
                    let mut server = CapacityServer::new(graph, customized);
                    total_time = total_time.add(init_start.elapsed());

                    // execute all queries
                    evaluation_breakpoints
                        .windows(2)
                        .map(|a| {
                            let mut current_idx = a[0];

                            while current_idx < a[1] {
                                // check if regular re-customization must be executed before query
                                if (current_idx + 1) % cl_update_frequency == 0 && current_idx as usize + 1 < queries.len() {
                                    let (_, time) = measure(|| {
                                        let customized = CustomizedCorridorLowerbound::new_from_capacity(&cch, &server.borrow_graph(), 72);
                                        server.customize(customized);
                                    });
                                    total_time = total_time.add(time);
                                    last_update_step = current_idx;
                                }

                                execute_query(
                                    &mut server,
                                    &potential_type.to_string(),
                                    &queries[current_idx as usize],
                                    current_idx as usize,
                                    &mut temp_time,
                                    &mut sum_dist,
                                    &mut num_runs,
                                    &mut total_time,
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
                                        total_time = total_time.add(time);
                                    }
                                }

                                // even if the update step violated some bounds, the result might still be valid
                                current_idx += server.result_valid() as u32;
                            }

                            EvaluatePotQualityEntry::new(potential_type.to_string(), a[1], total_time)
                        })
                        .collect::<Vec<EvaluatePotQualityEntry>>()
                }
                PotentialType::MultiMetrics => {
                    let mut last_update_step = 0;
                    // init server
                    let init_start = Instant::now();
                    let customized = CustomizedMultiMetrics::new_from_capacity(cch, &graph, &interval_pattern, mm_num_metrics as usize);
                    let mut server = CapacityServer::new(graph, customized);
                    total_time = total_time.add(init_start.elapsed());

                    // execute all queries
                    evaluation_breakpoints
                        .windows(2)
                        .map(|a| {
                            let mut current_idx = a[0];

                            while current_idx < a[1] {
                                // check if regular re-customization must be executed before query
                                if (current_idx + 1) % mm_update_frequency == 0 && current_idx as usize + 1 < queries.len() {
                                    let (_, time) = measure(|| server.customize(&interval_pattern, mm_num_metrics as usize));
                                    total_time = total_time.add(time);
                                    last_update_step = current_idx;
                                }

                                execute_query(
                                    &mut server,
                                    &potential_type.to_string(),
                                    &queries[current_idx as usize],
                                    current_idx as usize,
                                    &mut temp_time,
                                    &mut sum_dist,
                                    &mut num_runs,
                                    &mut total_time,
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
                                        total_time = total_time.add(time);
                                    }
                                }

                                // even if the update step violated some bounds, the result might still be valid
                                current_idx += server.result_valid() as u32;
                            }

                            EvaluatePotQualityEntry::new(potential_type.to_string(), a[1], total_time)
                        })
                        .collect::<Vec<EvaluatePotQualityEntry>>()
                }
            }
        })
        .collect::<Vec<EvaluatePotQualityEntry>>();

    // output data and write csv
    write_results(&results, &query_path.join("evaluate_potential_quality_unordered.csv"))
}

fn execute_query<Server: CapacityServerOps>(
    server: &mut Server,
    name: &str,
    query: &TDQuery<Timestamp>,
    idx: usize,
    time: &mut Duration,
    sum_dist: &mut u64,
    num_runs: &mut u64,
    total_time: &mut Duration,
) {
    let query_result = server.query_measured(query, true);
    let query_time = query_result
        .distance_result
        .time_potential
        .add(query_result.distance_result.time_query)
        .add(query_result.update_time);

    *time = time.add(query_time);
    *total_time = total_time.add(query_time);

    if let Some(distance) = query_result.query_result.map(|r| r.distance) {
        *sum_dist += distance as u64;
        *num_runs += 1;
    }

    if (idx + 1) % 1000 == 0 {
        println!(
            "{}: Finished {} queries. Last step: {}s - avg dist: {}, {} valid runs",
            name,
            idx + 1,
            time.as_secs_f64(),
            *sum_dist / max(*num_runs, 1),
            *num_runs
        );

        *time = Duration::ZERO;
        *sum_dist = 0;
        *num_runs = 0;
    }
}

fn write_results(results: &Vec<EvaluatePotQualityEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "name,query_count,time\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!("{},{},{}\n", entry.pot_name, entry.num_queries, entry.time.as_secs_f64());
        file.write(line.as_bytes())?;

        println!("------------------------------------");
        println!("Runtime Statistics for {} after {} queries", entry.pot_name, entry.num_queries);
        println!("Total time: {}s", entry.time.as_secs_f64());
    }

    Ok(())
}

/// Additional parameters: <path_to_graph> <path_to_queries> <num_buckets> <query_evaluation_frequency = 100000> <mm_num_metrics = 20> <mm_update_frequency = 50000> <cl_num_intervals = 72> <cl_update_frequency = 72>
fn parse_args() -> Result<(String, String, u32, u32, u32, u32, u32, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let num_buckets = parse_arg_required(&mut args, "number of buckets")?;

    let query_evaluation_frequency = parse_arg_optional(&mut args, 100000);
    let mm_num_metrics = parse_arg_optional(&mut args, 20);
    let mm_update_frequency = parse_arg_optional(&mut args, 50000);
    let cl_num_intervals = parse_arg_optional(&mut args, 72);
    let cl_update_frequency = parse_arg_optional(&mut args, 100000);

    assert!(query_evaluation_frequency > 0 && mm_num_metrics > 0 && mm_update_frequency > 0 && cl_num_intervals > 0 && cl_update_frequency > 0);

    Ok((
        graph_directory,
        query_directory,
        num_buckets,
        query_evaluation_frequency,
        mm_num_metrics,
        mm_update_frequency,
        cl_num_intervals,
        cl_update_frequency,
    ))
}

#[derive(Clone, Debug)]
struct EvaluatePotQualityEntry {
    pub pot_name: String,
    pub num_queries: u32,
    pub time: Duration,
}

impl EvaluatePotQualityEntry {
    pub fn new(pot_name: String, num_queries: u32, time: Duration) -> Self {
        Self { pot_name, num_queries, time }
    }
}
