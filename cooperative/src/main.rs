use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::potentials::td_backward_profile_potential::TDBackwardProfilePotential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::experiments::generate_queries::departure_distributions::{DepartureDistribution, UniformDeparture};
use cooperative::experiments::generate_queries::random_uniform::generate_random_uniform_td_queries;
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::cli::CliErr;
use rust_road_router::datastr::graph::Graph;
use rust_road_router::report::measure;

/// basic execution: run 1000 random queries, use cch-potentials
fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().ok_or(CliErr("No directory arg given"))?;
    let graph_directory = Path::new(arg);

    let (graph, time) = measure(|| load_capacity_graph(&graph_directory, 50, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (cch, time) = measure(|| {
        let order = load_node_order(graph_directory).unwrap();
        CCH::fix_order_and_build(&graph, order)
    });
    println!("CCH created in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (cch_pot_data, time) = measure(|| CCHPotData::new(&cch, &graph));
    println!("CCH customized in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let num_nodes = graph.num_nodes() as u32;

    let potential = cch_pot_data.forward_potential();

    let mut server = CapacityServer::new_with_potential(graph, potential);

    let queries = generate_random_uniform_td_queries(num_nodes as u32, 1000, UniformDeparture::new());

    let (_, time) = measure(|| {
        queries.iter().for_each(|query| {
            server.query(*query, true);
        })
    });
    println!("Queries took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    Ok(())
}
