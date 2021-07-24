use std::path::Path;

use rust_road_router::datastr::graph::Graph;
use rust_road_router::report::measure;

use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
//use crate::experiments::evaluation::evaluate_queries;
use crate::experiments::generate_queries::departure_distributions::{ConstantDeparture, DepartureDistribution};
use crate::experiments::generate_queries::random_uniform::generate_random_uniform_td_queries;
use crate::graph::traffic_functions::speed_functions::bpr_speed_function;
use crate::io::load_coords;
use crate::io::load_td_graph::load_td_capacity_graph;
use crate::visualization::generate_visualization_data;

pub fn run_td_server(graph_directory: &Path) {
    println!("Running TD server...");

    let (graph, time) = measure(||
        load_td_capacity_graph(&graph_directory, bpr_speed_function).unwrap()
    );
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let queries = generate_random_uniform_td_queries(
        graph.num_nodes() as u32,
        30,
        ConstantDeparture::new()
    );

    let mut server = CapacityServer::new(graph);

    //let result = evaluate_queries(&mut server, &queries);
    //dbg!(result);

    queries
        .iter()
        .for_each(|&query| {
            let result = server.query(query, true);

            if result.is_some() {
                generate_visualization_data(&result.unwrap().path.node_path, &lat, &lon);
            } else {
                println!(
                    "// Failed to find a path between {} [{},{}] and {} [{},{}]",
                    query.from,
                    lat[query.from as usize],
                    lon[query.from as usize],
                    query.to,
                    lat[query.to as usize],
                    lon[query.to as usize]
                );
            }
        });
}