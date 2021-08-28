use cooperative::dijkstra::potentials::cch_potential_init::init_cch_potential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::queries::{generate_queries, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
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
    let (graph, time) = measure(|| load_capacity_graph(graph_directory, num_buckets, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init potential
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);

    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());

    // initial run that serves as benchmark for future runs
    let initial_runtime = get_chunked_runtime_in_millis(&mut server, query_type.clone());
    println!("Initial Run (Benchmark): {} ms", initial_runtime);

    let mut runtime = initial_runtime;
    let mut num_runs = 1;

    while runtime < slowdown_factor * initial_runtime {
        num_runs += 1;
        runtime = get_chunked_runtime_in_millis(&mut server, query_type.clone());
        println!("Run {}: {} ms", num_runs, runtime);
    }

    println!("Total #queries until slowdown of {}: {}", slowdown_factor, num_runs * NUM_QUERIES_PER_RUN);

    Ok(())
}

fn get_chunked_runtime_in_millis<Pot: TDPotential>(server: &mut CapacityServer<Pot>, query_type: QueryType) -> f64 {
    let queries = generate_queries(server.borrow_graph(), query_type.clone(), NUM_QUERIES_PER_RUN);

    // TODO: measure time inside server (to neglect re-allocation effects on runtime)
    let (_, time) = measure(|| {
        queries.iter().for_each(|&query| {
            server.query(query, true);
        })
    });

    time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
}

fn parse_args() -> Result<(String, u32, f64, QueryType), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let slowdown_factor: f64 = parse_arg_required(&mut args, "slowdown factor")?;
    let query_mode = parse_arg_optional(&mut args, QueryType::Uniform);

    Ok((graph_directory, num_buckets, slowdown_factor, query_mode))
}
