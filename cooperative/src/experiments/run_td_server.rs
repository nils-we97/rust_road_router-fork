use std::path::Path;

use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::NodeId;
use rust_road_router::report::measure;

use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
use crate::experiments::evaluation::evaluate_queries;
//use crate::visualization::generate_visualization_data;
use crate::graph::traffic_functions::speed_functions::bpr_speed_function;
use crate::io::{load_coords, load_td_capacity_graph};

pub fn run_td_server(graph_directory: &Path) {
    let (graph, time) = measure(||
        load_td_capacity_graph(&graph_directory, bpr_speed_function).unwrap()
    );
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let mut server = CapacityServer::new(graph);

    let source = 169387;
    let target = 2625163;
    let num_queries = 15;

    println!("var source = [{}, {}];", lat[source as usize], lon[source as usize]);
    println!("var target = [{}, {}];", lat[target as usize], lon[target as usize]);

    let queries = (0..num_queries)
        .into_iter()
        .map(|_| TDQuery::new(source, target, 12345))
        .collect::<Vec<TDQuery<NodeId>>>();

    let result = evaluate_queries(&mut server, &queries);
    dbg!(result);

    /*for _ in 0..15 {
        let result = server.query(
            TDQuery::new(source, target, 12345),
            true,
        ).unwrap();

        generate_visualization_data(&result.path.node_path, &lat, &lon);
    }*/
}