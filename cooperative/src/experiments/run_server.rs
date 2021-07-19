use std::path::Path;

use rust_road_router::algo::{GenQuery, Query};
use rust_road_router::report::measure;

use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
use crate::graph::traffic_functions::time_functions::bpr_traffic_function;
use crate::io::{load_capacity_graph, load_coords};
use crate::visualization::generate_visualization_data;

pub fn run_server(graph_directory: &Path) {
    let (graph, time) = measure(||
        load_capacity_graph(&graph_directory, bpr_traffic_function).unwrap()
    );
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let mut server = CapacityServer::new(graph);

    let source = 169387;
    let target = 2625163;

    println!("var source = [{}, {}];", lat[source as usize], lon[source as usize]);
    println!("var target = [{}, {}];", lat[target as usize], lon[target as usize]);

    for _ in 0..15 {
        let result = server.query(
            Query::new(source, target, 0),
            true,
        ).unwrap();

        generate_visualization_data(&result.path.node_path, &lat, &lon);
    }
}