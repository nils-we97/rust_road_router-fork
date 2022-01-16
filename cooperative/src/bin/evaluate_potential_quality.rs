use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use cooperative::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use cooperative::dijkstra::potentials::TDPotential;
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
use rust_road_router::algo::TDQuery;
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
/// Compares the static lowerbound A* approach with the Corridor Lowerbound potential
/// Updates will occur after 50k, 100k, 200k queries (but are also configurable).
///
/// Additional parameters: <path_to_graph> <path_to_queries> <num_buckets> <corridor_lower_intervals> <query evaluation breakpoints, comma-separated> <cust-breakpoints = 100.000>
/// Note that all customization breakpoints must be divisible by all evaluation breakpoints
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, num_buckets, cl_num_intervals, evaluation_breakpoints, customization_breakpoints) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // verify correctness of customization and evaluation breakpoints
    evaluation_breakpoints[1..].iter().for_each(|&a| {
        customization_breakpoints.iter().for_each(|&b| {
            assert_eq!(
                a % b,
                0,
                "Cust-Breakpoints ({:?}) must be divisible by all Eval-Breakpoints ({:?}",
                evaluation_breakpoints,
                customization_breakpoints
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

    let results = [(false, *customization_breakpoints.last().unwrap())]
        .par_iter()
        .cloned()
        .chain(customization_breakpoints.par_iter().map(|&p| (true, p)))
        .flat_map(|(use_corridor_lowerbound, update_frequency)| {
            // load graph
            let mut graph = load_capacity_graph(&graph_path, num_buckets, bpr_traffic_function).unwrap();
            println!("{}-{}: Graph initialized!", update_frequency, use_corridor_lowerbound);

            let mut total_time_query = Duration::ZERO;
            let mut total_time_potential = Duration::ZERO;
            let mut total_time_update = Duration::ZERO;
            let mut total_time_reinit = Duration::ZERO;

            let mut statistics = Vec::new();

            if !use_corridor_lowerbound {
                for a in evaluation_breakpoints.windows(2) {
                    let mut start = a[0];
                    while start < a[1] {
                        // init server
                        let reinit_start = Instant::now();
                        let cch_pot_data = CCHPotData::new(&cch, &graph);
                        let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());
                        total_time_reinit = total_time_reinit.add(reinit_start.elapsed());

                        execute_queries(
                            &mut server,
                            &queries,
                            start,
                            start + update_frequency,
                            update_frequency,
                            use_corridor_lowerbound,
                            &mut total_time_potential,
                            &mut total_time_query,
                            &mut total_time_update,
                        );

                        // move graph out of server
                        let (g, _) = server.decompose();
                        graph = g;

                        statistics.push((
                            start + update_frequency,
                            update_frequency,
                            use_corridor_lowerbound,
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));

                        start += update_frequency;
                    }
                }
            } else {
                for a in evaluation_breakpoints.windows(2) {
                    let mut start = a[0];
                    while start < a[1] {
                        // re-init server
                        let reinit_start = Instant::now();
                        let customized = CustomizedApproximatedPeriodicTTF::new_from_capacity(&cch, &graph, cl_num_intervals);
                        let potential = CorridorLowerboundPotential::new(&customized);
                        let mut server = CapacityServer::new_with_potential(graph, potential);
                        total_time_reinit = total_time_reinit.add(reinit_start.elapsed());

                        execute_queries(
                            &mut server,
                            &queries,
                            start,
                            start + update_frequency,
                            update_frequency,
                            use_corridor_lowerbound,
                            &mut total_time_potential,
                            &mut total_time_query,
                            &mut total_time_update,
                        );

                        // move graph out of server
                        let (g, _) = server.decompose();
                        graph = g;

                        statistics.push((
                            start + update_frequency,
                            update_frequency,
                            use_corridor_lowerbound,
                            total_time_potential,
                            total_time_query,
                            total_time_update,
                            total_time_reinit,
                        ));

                        start += update_frequency;
                    }
                }
            }
            statistics
        })
        .collect::<Vec<(u32, u32, bool, Duration, Duration, Duration, Duration)>>();

    // output data and write csv
    write_results(&results, &query_path.join("evaluate_pot_quality.csv"))
}

fn execute_queries<Pot: TDPotential>(
    server: &mut CapacityServer<Pot>,
    queries: &Vec<TDQuery<u32>>,
    start: u32,
    end: u32,
    update_frequency: u32,
    use_corridor_lowerbound: bool,
    total_time_potential: &mut Duration,
    total_time_query: &mut Duration,
    total_time_update: &mut Duration,
) {
    let mut time_query = Duration::ZERO;
    let mut time_update = Duration::ZERO;
    let mut time_potential = Duration::ZERO;
    let mut sum_dist = 0;

    (start..end)
        .into_iter()
        .zip(queries[start as usize..end as usize].iter())
        .for_each(|(idx, query)| {
            if (idx + 1) % 10000 == 0 {
                println!(
                    "{}-{}: Finished {} of {} queries. Last step: {}ms pot init, {}ms query, {}ms ttf update - avg dist: {}",
                    update_frequency,
                    use_corridor_lowerbound,
                    idx + 1,
                    queries.len(),
                    time_potential.as_secs_f64() * 1000.0,
                    time_query.as_secs_f64() * 1000.0,
                    time_update.as_secs_f64() * 1000.0,
                    sum_dist / 10000
                );

                time_potential = Duration::ZERO;
                time_query = Duration::ZERO;
                time_update = Duration::ZERO;
                sum_dist = 0;
            }

            let query_result = server.query_measured(*query, true);
            time_potential = time_potential.add(query_result.distance_result.time_potential);
            time_query = time_query.add(query_result.distance_result.time_query);
            time_update = time_update.add(query_result.update_time);
            sum_dist += query_result.query_result.map(|x| x.distance).unwrap_or(0) as u64;

            *total_time_query = total_time_query.add(query_result.distance_result.time_query);
            *total_time_potential = total_time_potential.add(query_result.distance_result.time_potential);
            *total_time_update = total_time_update.add(query_result.update_time);
        });
}

fn write_results(data: &Vec<(u32, u32, bool, Duration, Duration, Duration, Duration)>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "query_count,update_frequency,time_potential,time_query,time_update,time_reinit\n";
    file.write(header.as_bytes())?;

    for (query_count, update_frequency, use_cl, time_pot, time_query, time_update, time_reinit) in data {
        let line = format!(
            "{},{},{},{},{},{},{}\n",
            query_count,
            update_frequency,
            use_cl,
            time_pot.as_secs_f64() * 1000.0,
            time_query.as_secs_f64() * 1000.0,
            time_update.as_secs_f64() * 1000.0,
            time_reinit.as_secs_f64() * 1000.0
        );
        file.write(line.as_bytes())?;

        println!("------------------------------------");
        println!("Runtime Statistics for {}-{} after {} queries", update_frequency, use_cl, query_count);
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
    let customization_breakpoints = parse_arg_optional(&mut args, "50000".to_string());

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
