use std::env;
use std::error::Error;

use cooperative::graph::preprocessing::largest_scc;
use rust_road_router::report::measure;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::graph::traffic_functions::time_functions::bpr_traffic_function;
use cooperative::io::load_coords;
use rust_road_router::io::{Deconstruct, Store};

//use cooperative::experiments::run_td_server::run_td_server;
//use cooperative::experiments::run_server::run_server;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("germany");

    let (graph, time) = measure(||
        load_capacity_graph(&graph_directory, bpr_traffic_function).unwrap()
    );
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (graph, lon, lat) = largest_scc(&graph, &lon, &lat);

    let update_directory = graph_directory.parent().unwrap().join("germany_neu");

    graph.deconstruct_to(&update_directory)?;
    lon.write_to(&update_directory.join("longitude"))?;
    lat.write_to(&update_directory.join("latitude"))?;

    //run_td_server(&graph_directory);

    Ok(())
}