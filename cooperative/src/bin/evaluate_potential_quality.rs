use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::balanced_interval_pattern;
use cooperative::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::permutate_queries;
use cooperative::experiments::types::PotentialType;
use cooperative::graph::traffic_functions::bpr_traffic_function;
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
    let temp_graph = load_capacity_graph(graph_path, num_buckets, bpr_traffic_function)?;

    let order = load_node_order(graph_path)?;
    let (cch, time) = measure(|| CCH::fix_order_and_build(&temp_graph, order));
    drop(temp_graph);
    println!("CCH created in {} ms", time.as_secs_f64() * 1000.0);

    let results = [(PotentialType::CCHPot, *customization_breakpoints.last().unwrap())]
        .par_iter()
        .cloned()
        .chain(customization_breakpoints.par_iter().map(|&p| (PotentialType::MultiMetrics, p)))
        //.chain(customization_breakpoints.iter().map(|&p| (PotentialType::CorridorLowerbound, p)))
        .flat_map(|(potential_type, update_frequency)| {
            let heuristic_name = format!("{}-{}", update_frequency, potential_type.to_string());

            // load graph
            let graph = load_capacity_graph(&graph_path, num_buckets, bpr_traffic_function).unwrap();
            println!("{}: Graph initialized!", heuristic_name);

            let mut total_time_query = Duration::ZERO;
            let mut total_time_potential = Duration::ZERO;
            let mut total_time_update = Duration::ZERO;
            let mut total_time_reinit = Duration::ZERO;

            let mut time_query = Duration::ZERO;
            let mut time_potential = Duration::ZERO;
            let mut time_update = Duration::ZERO;
            let mut sum_dist = 0;

            let mut statistics = Vec::new();

            match potential_type {
                PotentialType::CCHPot => {
                    let init_start = Instant::now();
                    let cch_pot_data = CCHPotData::new(&cch, &graph);
                    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
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
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );
                            // no re-customization required!
                        }

                        statistics.push((
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
                    /*let init_start = Instant::now();
                    let mut customized_ttf = CustomizedApproximatedPeriodicTTF::new_from_capacity(&cch, &graph, 72);
                    let mut customized_bounds = CustomizedLowerUpper::new(&cch, graph.travel_time());
                    customized_bounds.scale_upper_bounds();

                    let potential = CorridorLowerboundPotential::new_with_separate_bounds(&customized_ttf, &customized_bounds);
                    let mut server = CapacityServer::new_with_potential(graph, potential);
                    total_time_reinit = total_time_reinit.add(init_start.elapsed());

                    // execute all queries
                    for a in evaluation_breakpoints.windows(2) {
                        let mut current_idx = a[0];

                        while current_idx < a[1] {
                            execute_query(
                                &mut server,
                                heuristic_name.as_str(),
                                &queries[current_idx as usize],
                                current_idx as usize,
                                &mut time_potential,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            if (current_idx + 1) % update_frequency == 0 {
                                let reinit_start = Instant::now();
                                let (graph, pot) = server.decompose();
                                drop(pot);

                                let mut customized_ttf_update = CustomizedApproximatedPeriodicTTF::new_from_capacity(&cch, &graph, 72);
                                std::mem::swap(&mut customized_ttf.downward_intervals, &mut customized_ttf_update.downward_intervals);

                                customized_bounds = CustomizedLowerUpper::new(&cch, graph.travel_time());
                                customized_bounds.scale_upper_bounds();

                                let potential = CorridorLowerboundPotential::new_with_separate_bounds(&customized_ttf, &customized_bounds);
                                server = CapacityServer::new_with_potential(graph, potential);
                                total_time_reinit = total_time_reinit.add(reinit_start.elapsed());

                                current_idx += 1;
                            } else if server.requires_pot_update() {
                                customized_bounds = CustomizedLowerUpper::new(&cch, server.borrow_graph().travel_time());
                                customized_bounds.scale_upper_bounds();
                                let (_, time) = measure(|| server.update_potential_bounds(&customized_bounds));
                                total_time_reinit = total_time_reinit.add(time);
                            } else {
                                current_idx += 1;
                            }
                        }

                        statistics.push((
                            heuristic_name.clone(),
                            a[1],
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));
                    }*/
                }
                PotentialType::MultiMetrics => {
                    // init server
                    let init_start = Instant::now();
                    let customized = CustomizedMultiMetrics::new(
                        &cch,
                        graph.departure(),
                        graph.travel_time(),
                        &balanced_interval_pattern(),
                        mm_num_metrics as usize,
                        true,
                    );
                    let potential = MultiMetricPotential::new(customized);
                    let mut server = CapacityServer::new_with_potential(graph, potential);
                    total_time_reinit = total_time_reinit.add(init_start.elapsed());

                    // execute all queries
                    for a in evaluation_breakpoints.windows(2) {
                        let mut current_idx = a[0];

                        while current_idx < a[1] {
                            execute_query(
                                &mut server,
                                heuristic_name.as_str(),
                                &queries[current_idx as usize],
                                current_idx as usize,
                                &mut time_potential,
                                &mut time_query,
                                &mut time_update,
                                &mut sum_dist,
                                &mut total_time_potential,
                                &mut total_time_query,
                                &mut total_time_update,
                            );

                            if (current_idx + 1) % update_frequency == 0 {
                                let reinit_start = Instant::now();
                                let customized = CustomizedMultiMetrics::new(
                                    &cch,
                                    server.borrow_graph().departure(),
                                    server.borrow_graph().travel_time(),
                                    &balanced_interval_pattern(),
                                    mm_num_metrics as usize,
                                    true,
                                );
                                let potential = MultiMetricPotential::new(customized);
                                server.update_potential(potential);
                                total_time_reinit = total_time_reinit.add(reinit_start.elapsed());

                                current_idx += 1;
                            } else if server.requires_pot_update() {
                                let (_, time) = measure(|| server.update_potential_bounds());
                                total_time_reinit = total_time_reinit.add(time);
                            } else {
                                current_idx += 1;
                            }
                        }

                        statistics.push((
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
        .collect::<Vec<(String, u32, Duration, Duration, Duration, Duration)>>();

    // output data and write csv
    write_results(&results, &query_path.join("evaluate_pot_quality.csv"))
}

fn execute_query<Pot: TDPotential>(
    server: &mut CapacityServer<Pot>,
    name: &str,
    query: &TDQuery<Timestamp>,
    idx: usize,
    time_potential: &mut Duration,
    time_query: &mut Duration,
    time_update: &mut Duration,
    sum_dist: &mut u64,
    total_time_potential: &mut Duration,
    total_time_query: &mut Duration,
    total_time_update: &mut Duration,
) {
    let query_result = server.query_measured(*query, true);
    *time_potential = time_potential.add(query_result.distance_result.time_potential);
    *time_query = time_query.add(query_result.distance_result.time_query);
    *time_update = time_update.add(query_result.update_time);
    *sum_dist += query_result.query_result.map(|x| x.distance).unwrap_or(0) as u64;

    *total_time_query = total_time_query.add(query_result.distance_result.time_query);
    *total_time_potential = total_time_potential.add(query_result.distance_result.time_potential);
    *total_time_update = total_time_update.add(query_result.update_time);

    if (idx + 1) % 1000 == 0 {
        println!(
            "{}: Finished {} queries. Last step: {}s pot init, {}s query, {}s ttf update - avg dist: {}",
            name,
            idx + 1,
            time_potential.as_secs_f64(),
            time_query.as_secs_f64(),
            time_update.as_secs_f64(),
            *sum_dist / 1000
        );

        *time_potential = Duration::ZERO;
        *time_query = Duration::ZERO;
        *time_update = Duration::ZERO;
        *sum_dist = 0;
    }
}

fn write_results(results: &Vec<(String, u32, Duration, Duration, Duration, Duration)>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "name,query_count,time_potential,time_query,time_update,time_reinit\n";
    file.write(header.as_bytes())?;

    for (name, query_count, time_pot, time_query, time_update, time_reinit) in results {
        let line = format!(
            "{},{},{},{},{},{}\n",
            name,
            query_count,
            time_pot.as_secs_f64() * 1000.0,
            time_query.as_secs_f64() * 1000.0,
            time_update.as_secs_f64() * 1000.0,
            time_reinit.as_secs_f64() * 1000.0
        );
        file.write(line.as_bytes())?;

        println!("------------------------------------");
        println!("Runtime Statistics for {} after {} queries", name, query_count);
        println!(
            "Potential init: {}, query: {}, update: {}, reinit: {}",
            time_pot.as_secs_f64(),
            time_query.as_secs_f64(),
            time_update.as_secs_f64(),
            time_reinit.as_secs_f64()
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
