use rust_road_router::report::measure;

use crate::dijkstra::server::CapacityServerOps;
use crate::experiments::generate_queries::departure_distributions::{DepartureDistribution, UniformDeparture};
use crate::experiments::generate_queries::random_uniform::generate_random_uniform_td_queries;

const NUM_QUERIES_PER_RUN: u32 = 1000;

/**
 * Evaluate the quality of the given potential function w.r.t. the updates caused by each query.
 *
 * The `slowdown_factor` determines how slow the queries must become
 * (in average over `NUM_QUERIES_PER_RUN` queries) until the evaluation terminates.
 */
pub fn evaluate_potential_quality<Pot>(server: &mut impl CapacityServerOps<Pot>, num_nodes: u32, slowdown_factor: f64) -> u32 {
    assert!(slowdown_factor > 1.0, "Slowdown Factor must be greater than 1!");

    let initial_runtime = get_chunked_runtime_in_millis(server, num_nodes);
    println!("Initial Run (Benchmark): {} ms", initial_runtime);

    let mut runtime = initial_runtime;
    let mut num_runs = 1;

    while runtime < slowdown_factor * initial_runtime {
        num_runs += 1;
        runtime = get_chunked_runtime_in_millis(server, num_nodes);
        println!("Run {}: {} ms", num_runs, runtime);
    }

    num_runs * NUM_QUERIES_PER_RUN
}

fn get_chunked_runtime_in_millis<Pot>(server: &mut impl CapacityServerOps<Pot>, num_nodes: u32) -> f64 {
    let queries = generate_random_uniform_td_queries(num_nodes, NUM_QUERIES_PER_RUN, UniformDeparture::new());

    // exclude query generation time
    let (_, time) = measure(|| {
        queries.iter().for_each(|&query| {
            server.query(query, true);
        })
    });

    time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
}
