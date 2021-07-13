#![feature(iter_intersperse)]

use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::graph::capacity_graph::{CapacityGraph};
use rust_road_router::algo::{GenQuery, Query};
use rust_road_router::io::{Load};
use rust_road_router::report::measure;
use cooperative::graph::weight_functions::dummy_weight_function;
use cooperative::dijkstra::server::CapacityServer;
use cooperative::visualization::generate_visualization_data;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("germany");

    let (graph, time) = measure(|| load_graph(&graph_directory).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    /*
        let mut server: Server<CapacityGraph, DefaultOps, _, &CapacityGraph> = Server::new(&mut graph);
        let mut result = server.query(Query::new(0, 1123, 0)).unwrap();
        let path = result.path().iter().cloned().collect::<Vec<EdgeId>>();
        println!("{} {}", result.distance(), path_to_string(&path));
     */


    let mut server = CapacityServer::new(graph);

    let source = 1234567;
    let mut target = 2949124;

    for i in 0..lat.len() {
        if lat[i] > 53.0 {
            target = i as u32; break;
        }
    }

    println!("var source = [{}, {}];", lat[source as usize], lon[source as usize]);
    println!("var target = [{}, {}];", lat[target as usize], lon[target as usize]);

    for _ in 0..15 {
        let (_, path) = server.query(Query::new(source, target, 0)).unwrap();
        generate_visualization_data(&path, &lat, &lon);
        server.update(&path);
    }

    Ok(())
}

fn load_graph(graph_directory: &Path) -> Result<CapacityGraph, Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let weight = Vec::load_from(graph_directory.join("travel_time"))?;
    let capacity = vec![5; weight.len()];

    Ok(CapacityGraph::new(first_out, head, weight, capacity, dummy_weight_function))
}

fn load_coords(graph_directory: &Path) -> Result<(Vec<f32>, Vec<f32>), Box<dyn Error>> {
    let lon = Vec::load_from(graph_directory.join("longitude"))?;
    let lat= Vec::load_from(graph_directory.join("latitude"))?;

    Ok((lon, lat))
}