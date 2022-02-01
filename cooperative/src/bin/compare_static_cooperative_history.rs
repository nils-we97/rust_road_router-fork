use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::complete_balanced_interval_pattern;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::capacity_graph::CapacityGraph;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::{load_capacity_graph, load_used_speed_profiles};
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::query::Server as CCHServer;
use rust_road_router::algo::customizable_contraction_hierarchy::{customize, customize_perfect, DirectedCCH, CCH};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, EdgeIdT, FirstOutGraph, Graph, NodeId, Weight, INFINITY};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::ops::Add;
use std::path::Path;
use std::str::FromStr;
use std::time::{Duration, Instant};

/// -- main experiment on cooperative graphs, extending cooperative graphs with historic data --
///
/// Evaluate the difference between static and cooperative routing.
/// In a static setting, the edge weights are not updated after each query.
/// Instead, we simulate frequent updates by running a CCH customization each 20k/100k/.. steps.
/// For those customizations, the edge weights are taken from the cooperative graph with the highest bucket count.
/// Therefore, those calculations occur within the same thread.
///
/// In order to accelerate the queries, a Multi-Metric potential with default parameters is used
///
/// Additional parameters: <path_to_graph> <path_to_queries> <evaluation_frequency> <coop_bucket_counts> <coop_graph_history> <cch_update_frequencies=0,20000,100000> <pot_num_metrics=20> <pot_update_frequency=50000>

fn main() -> Result<(), Box<dyn Error>> {
    let (
        graph_directory,
        query_directory,
        evaluation_frequency,
        coop_bucket_counts,
        graph_history_directories,
        cch_update_frequencies,
        pot_num_metrics,
        pot_update_frequency,
    ) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    // load queries
    let queries = load_queries(&query_path)?;

    // load node order, init cch
    let temp_graph = load_capacity_graph(&graph_path, 1, BPRTrafficFunction::default())?;
    let order = load_node_order(&graph_path)?;
    let cch = CCH::fix_order_and_build(&temp_graph, order.clone());
    drop(temp_graph);

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

    // load/init additional structures
    let intervals = complete_balanced_interval_pattern();
    let mut results = Vec::new();

    println!("Starting to create server structs..");

    // initialize coop servers
    let mut servers = coop_bucket_counts
        .iter()
        .zip(graph_history_directories.iter())
        .map(|(&num_buckets, history_directory)| {
            // init graphs with expected speeds
            let mut graph = load_capacity_graph(&graph_path, num_buckets, BPRTrafficFunction::default()).unwrap();
            let historic_speeds = load_used_speed_profiles(&graph_path.join("speeds").join(history_directory)).unwrap();
            graph.add_historic_speeds(historic_speeds);

            let (customized, time_init) = measure(|| {
                let coop_cch = CCH::fix_order_and_build(&graph, order.clone());
                CustomizedMultiMetrics::new_from_capacity(coop_cch, &graph, &intervals, pot_num_metrics as usize)
            });

            let server = CapacityServer::new(graph, customized);
            CoopServerEntry::new(server, time_init)
        })
        .collect::<Vec<CoopServerEntry>>();

    // append cch servers to last coop server
    servers.last_mut().map(|entry| {
        entry.cch_servers = cch_update_frequencies
            .iter()
            .map(|&cch_update_frequency| {
                let (customized, init_time) = measure(|| {
                    let lower_bound = graph_at_timestamp(entry.server.borrow_graph(), 0);
                    customize_perfect(customize(&cch, &lower_bound))
                });
                CCHServerEntry::new(CCHServer::new(customized), cch_update_frequency, init_time)
            })
            .collect();
    });

    println!("Initialized all server structs, starting queries..");

    for a in evaluation_breakpoints.windows(2) {
        // parallel query execution for all servers
        servers.par_iter_mut().for_each(|entry| {
            (a[0] as usize..a[1] as usize)
                .into_iter()
                .zip(queries[a[0] as usize..a[1] as usize].iter())
                .for_each(|(idx, query)| {
                    if (idx + 1) % 10000 == 0 {
                        println!("-----------------");
                        println!(
                            "Coop-{}: Finished {} of {} queries",
                            entry.server.borrow_graph().num_buckets(),
                            idx + 1,
                            queries.len()
                        );
                        println!(
                            "Time: {}s customization, {}s queries",
                            entry.cust_time.as_secs_f64(),
                            entry.query_time.as_secs_f64()
                        );
                        if !entry.cch_servers.is_empty() {
                            println!(
                                "CCHs ({:?}): {:?} customization, {:?} query",
                                entry.cch_servers.iter().map(|e| e.cust_frequency).collect::<Vec<u32>>(),
                                entry.cch_servers.iter().map(|e| e.cust_time.as_secs_f64()).collect::<Vec<f64>>(),
                                entry.cch_servers.iter().map(|e| e.query_time.as_secs_f64()).collect::<Vec<f64>>(),
                            )
                        }
                        println!("-----------------");
                    }

                    // execute query on coop server
                    let mut coop_updated = false;

                    // check for regular customization of coop server
                    if (idx as u32 + 1) % pot_update_frequency == 0 {
                        let (_, time) = measure(|| entry.server.customize(&intervals, pot_num_metrics as usize));
                        entry.cust_time = entry.cust_time.add(time);
                        coop_updated = true;
                    }

                    // repeat query if it fails the first time, panic after second fail
                    loop {
                        let (coop_result, time) = measure(|| entry.server.query(query, true));
                        entry.query_time = entry.query_time.add(time);

                        // check if potential needs to be updated
                        if !entry.server.result_valid() || !entry.server.update_valid() {
                            if coop_updated {
                                // panic to avoid infinite loops
                                panic!("{} - failed twice in the same step!", &entry.type_name);
                            } else {
                                // re-customization of upper bounds
                                coop_updated = true;
                                println!("-- {} - potential update after {} steps", &entry.type_name, idx + 1);
                                let (_, time) = measure(|| entry.server.customize_upper_bound());
                                entry.cust_time = entry.cust_time.add(time);
                            }
                        }

                        if entry.server.result_valid() {
                            if let Some(result) = coop_result {
                                entry.query_paths.push(result.path.edge_path);
                                entry.query_departures.push(query.departure);
                            }
                            break;
                        }
                    }

                    // process queries on cch servers
                    entry.cch_servers.iter_mut().for_each(|cch_entry| {
                        // check if customization is required
                        if (idx + 1) as u32 % cch_entry.cust_frequency == 0 {
                            println!(
                                "Customizing CCH graph after {} queries (frequency: {}, timestamp: {})",
                                idx + 1,
                                cch_entry.cust_frequency,
                                query.departure
                            );

                            let (_, time) = measure(|| {
                                let cch_graph = graph_at_timestamp(entry.server.borrow_graph(), query.departure);
                                let customized = customize_perfect(customize(&cch, &cch_graph));
                                cch_entry.server.update(customized);
                            });
                            cch_entry.cust_time = cch_entry.cust_time.add(time);
                        }

                        // execute query and re-build path
                        let (result, time) = measure(|| {
                            cch_entry.server.query(Query::new(query.from, query.to, 0)).node_path().map(|path| {
                                path.windows(2)
                                    .map(|edge| {
                                        entry
                                            .server
                                            .borrow_graph()
                                            .edge_indices(edge[0], edge[1])
                                            .min_by_key(|&EdgeIdT(e)| entry.server.borrow_graph().free_flow_time()[e as usize])
                                            .map(|EdgeIdT(e)| e)
                                            .unwrap()
                                    })
                                    .collect::<Vec<EdgeId>>()
                            })
                        });
                        cch_entry.query_time = cch_entry.query_time.add(time);

                        if let Some(edge_path) = result {
                            cch_entry.query_paths.push(edge_path);
                            cch_entry.query_departures.push(query.departure);
                        }
                    });
                });
        });

        // evaluate the results on the server with the highest bucket count
        let evaluation_server = servers.last().map(|e| &e.server).unwrap();
        debug_assert_eq!(
            evaluation_server.borrow_graph().num_buckets(),
            coop_bucket_counts.iter().max().cloned().unwrap()
        );

        let evaluation_start = Instant::now();
        let current_results = servers
            .par_iter()
            .flat_map(|entry| {
                let mut temp_results = Vec::new();

                // start with cooperative results
                let coop_dist = sum_path_distances(evaluation_server, &entry.query_paths, &entry.query_departures);

                println!("------------------------------------------");
                println!(
                    "Cooperative Statistics ({} buckets) after {} runs:",
                    entry.server.borrow_graph().num_buckets(),
                    a[1]
                );
                println!(
                    "Customization: {}s, Query: {}s, total distance: {} ({} runs -> avg: {})",
                    entry.cust_time.as_secs_f64(),
                    entry.query_time.as_secs_f64(),
                    coop_dist,
                    entry.query_departures.len(),
                    coop_dist / entry.query_departures.len() as u64,
                );

                temp_results.push(CompareStaticCooperativeStatisticEntry::new(
                    entry.type_name.clone(),
                    entry.query_time,
                    entry.cust_time,
                    a[1],
                    entry.query_departures.len() as u32,
                    coop_dist,
                    coop_dist / entry.query_departures.len() as u64,
                ));

                // proceed with cch results
                let cch_results = entry
                    .cch_servers
                    .par_iter()
                    .map(|cch_entry| {
                        let cch_dist = sum_path_distances(evaluation_server, &cch_entry.query_paths, &cch_entry.query_departures);

                        println!("------------------------------------------");
                        println!("CCH Statistics (update frequency: {}) after {} runs:", cch_entry.cust_frequency, a[1]);
                        println!(
                            "Customization: {}s, Query: {}s, total distance: {} ({} runs -> avg: {})",
                            cch_entry.cust_time.as_secs_f64(),
                            cch_entry.query_time.as_secs_f64(),
                            cch_dist,
                            cch_entry.query_departures.len(),
                            cch_dist / cch_entry.query_departures.len() as u64,
                        );

                        CompareStaticCooperativeStatisticEntry::new(
                            cch_entry.type_name.clone(),
                            cch_entry.query_time,
                            cch_entry.cust_time,
                            a[1],
                            cch_entry.query_departures.len() as u32,
                            cch_dist,
                            cch_dist / cch_entry.query_departures.len() as u64,
                        )
                    })
                    .collect::<Vec<CompareStaticCooperativeStatisticEntry>>();

                temp_results.extend_from_slice(&cch_results);
                temp_results
            })
            .collect::<Vec<CompareStaticCooperativeStatisticEntry>>();

        println!("------------------------------------------");
        println!("Evaluation took {}s", evaluation_start.elapsed().as_secs_f64());

        results.extend_from_slice(&current_results);
    }

    write_results(&results, &query_path)
}

fn write_results(results: &Vec<CompareStaticCooperativeStatisticEntry>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(&path.join("compare_static_cooperative_history.csv"))?;

    let header = "type,cust_time,query_time,num_runs,num_actual_runs,total_dist,avg_dist\n";
    file.write(header.as_bytes())?;

    for entry in results {
        let line = format!(
            "{},{},{},{},{},{},{}\n",
            entry.query_type,
            entry.customization_time.as_secs_f64(),
            entry.query_time.as_secs_f64(),
            entry.num_runs,
            entry.num_actual_runs,
            entry.total_dist,
            entry.avg_dist
        );
        file.write(line.as_bytes())?;
    }

    Ok(())
}

fn graph_at_timestamp(graph: &CapacityGraph, ts: Timestamp) -> FirstOutGraph<&[EdgeId], &[NodeId], Vec<Weight>> {
    let weights = (0..graph.num_arcs() as EdgeId)
        .map(|e| graph.travel_time_function(e).eval(ts))
        .collect::<Vec<Weight>>();
    FirstOutGraph::new(graph.first_out(), graph.head(), weights)
}

fn sum_path_distances(evaluation_server: &CapacityServer<CustomizedMultiMetrics>, paths: &Vec<Vec<EdgeId>>, departures: &Vec<Timestamp>) -> u64 {
    debug_assert_eq!(paths.len(), departures.len());

    paths
        .iter()
        .zip(departures.iter())
        .map(|(path, &departure)| {
            Some(evaluation_server.path_distance(path, departure))
                .filter(|&dist| dist != INFINITY)
                .map(|dist| dist as u64)
                .unwrap_or(0)
        })
        .sum::<u64>()
}

fn parse_args() -> Result<(String, String, u32, Vec<u32>, Vec<String>, Vec<u32>, u32, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let evaluation_frequency: u32 = parse_arg_required(&mut args, "Evaluation Frequency")?;
    let bucket_counts: String = parse_arg_required(&mut args, "Bucket Counts")?;
    let graph_history: String = parse_arg_required(&mut args, "Coop Graph Expected Speeds")?;
    let cch_update_frequencies = parse_arg_optional(&mut args, "0,20000,100000".to_string());
    let pot_num_metrics = parse_arg_optional(&mut args, 20);
    let pot_update_frequency = parse_arg_optional(&mut args, 50000);

    let bucket_counts = bucket_counts.split(",").filter_map(|val| u32::from_str(val).ok()).collect::<Vec<u32>>();
    let graph_history = graph_history.split(",").map(|s| s.to_string()).collect::<Vec<String>>();
    let mut cch_update_frequencies = cch_update_frequencies
        .split(",")
        .filter_map(|val| u32::from_str(val).ok())
        .map(|val| if val == 0 { INFINITY } else { val })
        .collect::<Vec<u32>>();

    assert!(!bucket_counts.is_empty() && !cch_update_frequencies.is_empty() && evaluation_frequency > 0);
    assert!(
        bucket_counts.windows(2).all(|a| a[0] < a[1]),
        "Bucket counts must be sorted in ascending order!"
    );

    // sort and remove duplicates
    cch_update_frequencies.sort();
    cch_update_frequencies.dedup();

    Ok((
        graph_directory,
        query_directory,
        evaluation_frequency,
        bucket_counts,
        graph_history,
        cch_update_frequencies,
        pot_num_metrics,
        pot_update_frequency,
    ))
}

#[derive(Clone)]
struct CompareStaticCooperativeStatisticEntry {
    pub query_type: String,
    pub query_time: Duration,
    pub customization_time: Duration,
    pub num_runs: u32,
    pub num_actual_runs: u32,
    pub total_dist: u64,
    pub avg_dist: u64,
}

impl CompareStaticCooperativeStatisticEntry {
    pub fn new(
        query_type: String,
        query_time: Duration,
        customization_time: Duration,
        num_runs: u32,
        num_actual_runs: u32,
        total_dist: u64,
        avg_dist: u64,
    ) -> Self {
        Self {
            query_type,
            query_time,
            customization_time,
            num_runs,
            num_actual_runs,
            total_dist,
            avg_dist,
        }
    }
}

struct CoopServerEntry {
    pub server: CapacityServer<CustomizedMultiMetrics>,
    pub cust_time: Duration,
    pub query_time: Duration,
    pub cch_servers: Vec<CCHServerEntry>,
    pub query_paths: Vec<Vec<EdgeId>>,
    pub query_departures: Vec<Timestamp>,
    pub type_name: String,
}

impl CoopServerEntry {
    pub fn new(server: CapacityServer<CustomizedMultiMetrics>, init_time: Duration) -> Self {
        let type_name = format!("coop-{}", server.borrow_graph().num_buckets());

        Self {
            server,
            cust_time: init_time,
            query_time: Duration::ZERO,
            cch_servers: vec![],
            query_paths: vec![],
            query_departures: vec![],
            type_name,
        }
    }
}

struct CCHServerEntry {
    pub server: CCHServer<DirectedCCH, DirectedCCH>,
    pub cust_frequency: u32,
    pub cust_time: Duration,
    pub query_time: Duration,
    pub query_paths: Vec<Vec<EdgeId>>,
    pub query_departures: Vec<Timestamp>,
    pub type_name: String,
}

impl CCHServerEntry {
    pub fn new(server: CCHServer<DirectedCCH, DirectedCCH>, cust_frequency: u32, init_time: Duration) -> Self {
        let type_name = format!("cch-{}", cust_frequency);

        Self {
            server,
            cust_frequency,
            cust_time: init_time,
            query_time: Duration::ZERO,
            query_paths: vec![],
            query_departures: vec![],
            type_name,
        }
    }
}
