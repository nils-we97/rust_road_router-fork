/*use std::path::Path;

use rust_road_router::report::measure;

use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
use crate::experiments::queries::population_density_based::generate_uniform_population_density_based_queries;
use crate::graph::traffic_functions::time_functions::bpr_traffic_function;
use crate::io::io_graph::load_capacity_graph;
use crate::io::load_coords;
use crate::io::population_grid::load_population_grid;
use crate::visualization::generate_visualization_data;
use rust_road_router::algo::a_star::ZeroPotential;
use crate::graph::capacity_graph::CapacityGraph;

pub fn run_server(graph_directory: &Path, population_directory: &Path) {
    let (graph, time) = measure(|| load_capacity_graph(&graph_directory, bpr_traffic_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((grid, population), time) = measure(|| load_population_grid(&population_directory).unwrap());
    println!(
        "Population structures initialized in {} ms",
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
    );

    let mut server = CapacityServer::<CapacityGraph, ZeroPotential>::new(graph);

    generate_uniform_population_density_based_queries(&lon, &lat, &grid, &population, 30)
        .iter()
        .for_each(|query| {
            let result = server.query(*query, true);

            if result.is_some() {
                generate_visualization_data(&result.unwrap().path.node_path, &lat, &lon);
            } else {
                print!("//Failed to find path between {} and {}", query.from, query.to);
            }
        });
}*/
