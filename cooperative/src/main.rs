use std::error::Error;
use std::env;

use rust_road_router::cli::CliErr;
use std::path::Path;
use rust_road_router::datastr::graph::{WeightedGraphReconstructor, NodeId, EdgeId, Weight, FirstOutGraph, Graph, OwnedGraph};
use rust_road_router::io::{ReconstructPrepared, Load};
use rust_road_router::report::measure;
use rust_road_router::algo::dijkstra::{Server, DijkstraOps, DefaultOps};
use rust_road_router::algo::{QueryServer, Query, GenQuery};
use rust_road_router::algo::a_star::ZeroPotential;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hello world");

    let (graph, time) = measure(|| load_graph().unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let mut server: Server<OwnedGraph, DefaultOps, _, &OwnedGraph> = Server::new(&graph);
    let result = server.query(Query::new(0, 1, 0)).unwrap();

    println!("{}", result.distance());

    Ok(())
}

fn load_graph() -> Result<OwnedGraph, Box<dyn Error>> {
    let graph_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("germany");

    println!("{}", graph_directory.as_path().to_str().unwrap());

    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let weight = Vec::load_from(graph_directory.join("travel_time"))?;

    Ok(OwnedGraph::new(first_out, head, weight))
}