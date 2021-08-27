/*use std::path::Path;

use rust_road_router::algo::a_star::ZeroPotential;
use rust_road_router::datastr::graph::Graph;
use rust_road_router::report::measure;

use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
use crate::experiments::evaluate_td_impact::evaluate_time_dependent_impact;
use crate::experiments::queries::departure_distributions::{ConstantDeparture, DepartureDistribution};
use crate::experiments::queries::random_uniform::generate_random_uniform_td_queries;
use crate::graph::td_capacity_graph::TDCapacityGraph;
use crate::graph::traffic_functions::speed_functions::bpr_speed_function;
use crate::io::io_td_graph::load_td_capacity_graph;

pub fn run_td_server(graph_directory: &Path) {
    println!("Running TD server...");

    let (td_graph_single_bucket, time) = measure(||
        load_td_capacity_graph(
            &graph_directory,
            1,
            bpr_speed_function,
        ).unwrap()
    );
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (td_graph, time) = measure(||
        load_td_capacity_graph(
            &graph_directory,
            30,
            bpr_speed_function,
        ).unwrap()
    );
    println!("TD-Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let queries = generate_random_uniform_td_queries(
        td_graph_single_bucket.num_nodes() as u32,
        500,
        ConstantDeparture::new(),
    );

    let mut server = CapacityServer::<TDCapacityGraph, ZeroPotential>::new(td_graph_single_bucket);
    let mut td_server = CapacityServer::<TDCapacityGraph, ZeroPotential>::new(td_graph);

    let result = evaluate_time_dependent_impact(
        &mut server,
        &mut td_server,
        &queries,
    );

    //let result = compare_static_cooperative(&mut server, &queries);
    dbg!(result);
}
*/
