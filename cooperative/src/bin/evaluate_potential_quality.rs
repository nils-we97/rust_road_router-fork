use cooperative::dijkstra::elimination_tree::approximated_periodic_ttf::customized::CustomizedApproximatedPeriodicTTF;
use cooperative::dijkstra::elimination_tree::multi_level_buckets::customized::CustomizedMultiLevels;
use cooperative::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use cooperative::dijkstra::potentials::multi_level_bucket_potential::CCHMultiLevelBucketPotential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::{generate_queries, QueryType};
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::io::io_graph::{load_used_capacity_graph, store_capacity_buckets};
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::Graph;
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::ops::Add;
use std::path::Path;

const NUM_QUERIES_PER_RUN: u32 = 1000;

/// Evaluates the quality of A*-Potentials with a rising number of queries.
/// As the potentials are calculated in advance (e.g. on a lowerbound graph), its quality will
/// get worse with with every additional query.
///
/// The algorithm will terminate as soon as the query runtime exceeds a given order of magnitude
/// in comparison with the first run.
/// To neglect random effects, each run consists of 1000 queries.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// Additional parameters: <path_to_graph> <num_buckets> <slowdown_factor> <query_mode = uniform>
/// <slowdown_factor> must be greater than 1 (and shouldn't be much larger than 10)
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, slowdown_factor, query_type) = parse_args()?;
    let graph_directory = Path::new(&path);

    assert!(slowdown_factor > 1.0, "Slowdown Factor must be greater than 1!");

    // load graph
    //let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    let (graph, time) = measure(|| load_used_capacity_graph(graph_directory, num_buckets, bpr_traffic_function, "300_1635950540").unwrap());

    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init potential
    let order = load_node_order(graph_directory)?;

    let (cch, time) = measure(|| CCH::fix_order_and_build(&graph, order));
    println!("CCH created in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // complete initialization block for static lowerbound pot, uncomment if not needed
    let (cch_pot_data, time) = measure(|| CCHPotData::new(&cch, &graph));
    println!("CCH customized in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
    let potential = cch_pot_data.forward_potential();

    // complete initialization block for MultiLevelBucket pot
    /*let (customized, time) = measure(|| {
        CustomizedMultiLevels::new(
            &cch,
            graph.departure(),
            graph.travel_time(),
            &vec![86_400_000 / 12, 86_400_000 / 48],
            graph.num_arcs() as u64 * 2000,
        )
    });
    println!(
        "Multi-Level-Bucket-CCH customized in {} ms",
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );
    let potential = CCHMultiLevelIntervalPotential::new_forward(&customized, 2);*/

    // complete initialization block for CorridorLowerbound pot
    /*let (customized, time) = measure(|| CustomizedApproximatedPeriodicTTF::new(&cch, graph.departure(), graph.travel_time(), 200, 96));
    println!(
        "Approximated Shortcut TTFs created in {} ms",
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );
    let potential = CorridorLowerboundPotential::new(&customized);*/

    // experimental init block for ALT context, currently unused
    /*let (alt_context, time) = measure(|| HeuristicUpperBoundALTPotentialContext::init(graph.first_out().to_vec(), graph.head().to_vec(), graph.travel_time()));
    println!("Initializing ALT context took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);*/

    let mut server = CapacityServer::new_with_potential(graph, potential);
    //let mut server = CapacityServer::<ZeroPotential>::new(graph);

    // initial run that serves as benchmark for future runs
    let (initial_runtime, time_potentials, time_queries, time_buckets, time_ttf) = get_chunked_runtime_in_millis(&mut server, query_type.clone());
    println!(
        "Initial Run (Benchmark): {} ms (Potentials: {}, Queries: {}, Buckets: {}, TTF: {})",
        initial_runtime, time_potentials, time_queries, time_buckets, time_ttf
    );

    let mut runtime = initial_runtime;
    let mut num_runs = 1;

    while runtime < slowdown_factor * initial_runtime {
        num_runs += 1;
        let (total_time, time_potentials, time_queries, time_buckets, time_ttf) = get_chunked_runtime_in_millis(&mut server, query_type.clone());
        println!(
            "Run {}: {} ms (Potentials: {}, Queries: {}, Buckets: {}, TTF: {})",
            num_runs, total_time, time_potentials, time_queries, time_buckets, time_ttf
        );
        runtime = total_time;
    }

    println!("Total #queries until slowdown of {}: {}", slowdown_factor, num_runs * NUM_QUERIES_PER_RUN);

    let out_name = format!("{}_{}", num_buckets, time::get_time().sec);
    store_capacity_buckets(server.borrow_graph(), graph_directory, out_name)
}

fn get_chunked_runtime_in_millis<Pot: TDPotential>(server: &mut CapacityServer<Pot>, query_type: QueryType) -> (f64, f64, f64, f64, f64) {
    let queries = generate_queries(server.borrow_graph(), query_type.clone(), NUM_QUERIES_PER_RUN);

    let mut time_potentials = time::Duration::zero();
    let mut time_queries = time::Duration::zero();
    let mut time_buckets = time::Duration::zero();
    let mut time_ttfs = time::Duration::zero();

    let mut num_relaxed_arcs = 0u32;
    let mut num_queue_pops = 0u32;

    let (_, total_time) = measure(|| {
        queries.iter().for_each(|&query| {
            let query_result = server.query_measured(query, true);

            time_potentials = time_potentials.add(query_result.distance_result.time_potential);
            time_queries = time_queries.add(query_result.distance_result.time_query);
            time_buckets = time_buckets.add(query_result.update_result.time_bucket_updates);
            time_ttfs = time_ttfs.add(query_result.update_result.time_ttf);

            num_relaxed_arcs += query_result.distance_result.num_relaxed_arcs;
            num_queue_pops += query_result.distance_result.num_queue_pops;
        })
    });

    println!(
        "Relaxed arcs: {} (avg: {}), Queue pops: {} (avg: {})",
        num_relaxed_arcs,
        num_relaxed_arcs / queries.len() as u32,
        num_queue_pops,
        num_queue_pops / queries.len() as u32
    );
    (
        total_time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_potentials.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_queries.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_buckets.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_ttfs.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
    )
}

fn parse_args() -> Result<(String, u32, f64, QueryType), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let slowdown_factor: f64 = parse_arg_required(&mut args, "slowdown factor")?;
    let query_mode = parse_arg_optional(&mut args, QueryType::Uniform);

    Ok((graph_directory, num_buckets, slowdown_factor, query_mode))
}
