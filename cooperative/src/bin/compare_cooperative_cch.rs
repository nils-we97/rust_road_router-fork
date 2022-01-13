use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::query::Server as CCHServer;
use rust_road_router::algo::customizable_contraction_hierarchy::{customize, customize_perfect, DirectedCCH, CCH};
use rust_road_router::algo::{GenQuery, Query, QueryServer};
use rust_road_router::datastr::graph::{EdgeIdGraph, EdgeIdT, FirstOutGraph, Graph, OwnedGraph};
use rust_road_router::io::Load;
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::ops::Add;
use std::path::Path;
use std::str::FromStr;
use std::time::Duration;

/// compares the cooperative routing approach with frequent routing in a CCH
///
/// additional parameters: <path_to_graph> <path_to_queries> <num_buckets> <evaluation_breakpoints> <coop-update-frequency> <CCH-update-frequency = 20000,100000>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, num_buckets, evaluation_breakpoints, _coop_frequency, cch_frequencies) = parse_args()?;

    let graph_path = Path::new(&graph_directory);
    let query_path = graph_path.join("queries").join(&query_directory);

    let queries = load_queries(&query_path)?;

    // init capacity graph for cooperative routing
    let coop_graph = load_capacity_graph(graph_path, num_buckets, bpr_traffic_function)?;

    // init cch
    let first_out = Vec::<u32>::load_from(&graph_path.join("first_out"))?;
    let head = Vec::<u32>::load_from(&graph_path.join("head"))?;
    let weight = Vec::<u32>::load_from(&graph_path.join("travel_time"))?;
    let weight = weight.iter().map(|&val| val * 1000).collect::<Vec<u32>>();
    let lower_bound = OwnedGraph::new(first_out, head, weight);

    let order = load_node_order(&graph_path)?;
    let cch = CCH::fix_order_and_build(&lower_bound, order);
    let cch_pot_data = CCHPotData::new(&cch, &lower_bound);

    // init coop server
    let mut server = CapacityServer::new_with_potential(coop_graph, cch_pot_data.forward_potential());

    // init cch server
    let mut cch_servers = cch_frequencies
        .iter()
        .map(|_| {
            let graph = lower_bound.clone();
            let customized = customize_perfect(customize(&cch, &graph));
            CCHServer::new(customized)
        })
        .collect::<Vec<CCHServer<DirectedCCH, DirectedCCH>>>();

    // run queries
    for a in evaluation_breakpoints.windows(2) {
        let mut total_time_coop = Duration::ZERO;
        let mut runs_coop = Vec::with_capacity(queries.len());
        let mut paths_coop = Vec::with_capacity(queries.len());

        let mut total_time_cch = vec![Duration::ZERO; cch_frequencies.len()];
        let mut cust_time_cch = vec![Duration::ZERO; cch_frequencies.len()];
        let mut runs_cch = vec![Vec::with_capacity(queries.len()); cch_frequencies.len()];
        let mut paths_cch = vec![Vec::with_capacity(queries.len()); cch_frequencies.len()];

        (a[0] as usize..a[1] as usize)
            .into_iter()
            .zip(queries[a[0] as usize..a[1] as usize].iter())
            .for_each(|(idx, query)| {
                if (idx + 1) % 10000 == 0 {
                    println!("-----------------");
                    println!("Finished {} of {} queries", idx + 1, queries.len());
                    println!(
                        "Coop: {} seconds, CCHs ({:?}): {:?} seconds each",
                        total_time_coop.as_secs_f64(),
                        &cch_frequencies,
                        total_time_cch.iter().map(|time| time.as_secs_f64()).collect::<Vec<f64>>()
                    );
                    println!("-----------------");
                }

                // process cooperative query
                let (coop_result, time) = measure(|| server.query(*query, true));
                total_time_coop = total_time_coop.add(time);
                if let Some(result) = coop_result {
                    runs_coop.push(idx);
                    paths_coop.push(result.path.edge_path);
                }

                // todo cooperative potential update?

                // process query for cch servers
                for i in 0..cch_frequencies.len() {
                    if idx > 0 && idx as u32 % cch_frequencies[i] == 0 {
                        let current_ts = queries[idx].departure;
                        println!(
                            "Customizing CCH graph after {} queries (frequency: {}), timestamp: {}",
                            idx, cch_frequencies[i], current_ts
                        );

                        let (customized, time) = measure(|| {
                            let coop_graph_borrow = server.borrow_graph();

                            let weights = (0..coop_graph_borrow.num_arcs() as u32)
                                .into_iter()
                                .map(|e| coop_graph_borrow.travel_time_function(e).eval(current_ts))
                                .collect::<Vec<u32>>();

                            let graph = FirstOutGraph::new(coop_graph_borrow.first_out(), coop_graph_borrow.head(), weights);
                            customize_perfect(customize(&cch, &graph))
                        });
                        cust_time_cch[i] = cust_time_cch[i].add(time);

                        cch_servers[i].update(customized);
                    }

                    let (path, time) = measure(|| {
                        cch_servers[i].query(Query::new(query.from, query.to, 0)).node_path().map(|node_path| {
                            node_path
                                .windows(2)
                                .map(|edge| {
                                    server
                                        .borrow_graph()
                                        .edge_indices(edge[0], edge[1])
                                        .min_by_key(|&EdgeIdT(e)| server.borrow_graph().free_flow_time()[e as usize])
                                        .map(|EdgeIdT(e)| e)
                                        .unwrap()
                                })
                                .collect::<Vec<u32>>()
                        })
                    });
                    total_time_cch[i] = total_time_cch[i].add(time);

                    if let Some(path) = path {
                        paths_cch[i].push(path);
                        runs_cch[i].push(idx);
                    }
                }
            });

        // Evaluate paths
        let coop_distance = paths_coop
            .iter()
            .zip(runs_coop.iter())
            .map(|(path, &query_idx)| server.path_distance(path, queries[query_idx].departure) as u64)
            .sum::<u64>();

        let cch_distances = paths_cch
            .iter()
            .enumerate()
            .map(|(idx, paths)| {
                paths
                    .iter()
                    .zip(runs_cch[idx].iter())
                    .map(|(path, &query_idx)| server.path_distance(path, queries[query_idx].departure) as u64)
                    .sum::<u64>()
            })
            .collect::<Vec<u64>>();

        println!("------------------------------------------");
        println!("Cooperative Statistics after {} runs:", a[1]);
        println!(
            "Time: {}s, total distance: {} ({} runs -> avg: {})",
            total_time_coop.as_secs_f64(),
            coop_distance,
            runs_coop.len(),
            coop_distance / runs_coop.len() as u64
        );

        for i in 0..cch_frequencies.len() {
            println!("------------------------------------------");
            println!("CCH Statistics after {} runs (customization each {} queries):", a[1], cch_frequencies[i]);
            println!(
                "Time: query {}s, customization {}s, total distance: {} ({} runs -> avg: {})",
                total_time_cch[i].as_secs_f64(),
                cust_time_cch[i].as_secs_f64(),
                cch_distances[i],
                runs_cch[i].len(),
                cch_distances[i] / runs_cch[i].len() as u64
            );
        }
        println!("------------------------------------------");
    }

    Ok(())
}

fn parse_args() -> Result<(String, String, u32, Vec<u32>, u32, Vec<u32>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let num_buckets = parse_arg_required(&mut args, "num buckets")?;
    let evaluation_breakpoints: String = parse_arg_required(&mut args, "Evaluation Breakpoints")?;
    let coop_update_frequency = parse_arg_required(&mut args, "Coop Update Frequency")?;
    let cch_update_frequency = parse_arg_optional(&mut args, "20000,100000".to_string());

    let mut cch_frequency = cch_update_frequency.split(",").filter_map(|val| u32::from_str(val).ok()).collect::<Vec<u32>>();

    let mut evaluation_breakpoints = ["0"]
        .iter()
        .cloned()
        .chain(evaluation_breakpoints.split(","))
        .filter_map(|val| u32::from_str(val).ok())
        .collect::<Vec<u32>>();

    assert!(!cch_frequency.is_empty() && !evaluation_breakpoints.is_empty());

    // sort and remove duplicates
    cch_frequency.sort();
    cch_frequency.dedup();
    evaluation_breakpoints.sort();
    evaluation_breakpoints.dedup();

    Ok((
        graph_directory,
        query_directory,
        num_buckets,
        evaluation_breakpoints,
        coop_update_frequency,
        cch_frequency,
    ))
}
