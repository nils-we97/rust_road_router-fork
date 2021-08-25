use std::path::Path;

use rust_road_router::datastr::graph::Graph;
//use rust_road_router::algo::ch_potentials::{CCHPotData, CCHPotential};
//use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
//use rust_road_router::datastr::graph::{EdgeId, FirstOutGraph, Graph, NodeId, Weight};
//use crate::graph::td_capacity_graph::TDCapacityGraph;
//use crate::io::io_node_order::load_node_order;
use rust_road_router::report::measure;

use crate::dijkstra::potentials::td_backward_profile_potential::TDBackwardProfilePotential;
use crate::dijkstra::server::{CapacityServer, CapacityServerOps};
use crate::experiments::evaluate_potential_quality::evaluate_potential_quality;
use crate::graph::traffic_functions::speed_functions::bpr_speed_function;
use crate::io::io_td_graph::load_td_capacity_graph;

pub fn run_server_with_potentials(graph_directory: &Path) {
    let (graph, time) = measure(|| load_td_capacity_graph(&graph_directory, 50, bpr_speed_function).unwrap());
    println!("Graph loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    /*let ((lon, lat), time) = measure(|| load_coords(&graph_directory).unwrap());
    println!("Coordinates loaded in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);*/

    /*let (cch, time) = measure(|| {
        let order = load_node_order(graph_directory).unwrap();
        CCH::fix_order_and_build(&graph, order)
    });
    println!("CCH created in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (cch_pot_data, time) = measure(|| CCHPotData::new(&cch, &graph));
    println!("CCH customized in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);*/

    let num_nodes = graph.num_nodes() as u32;

    let potential = TDBackwardProfilePotential::new(&graph);

    let mut server = CapacityServer::new_with_potential(graph, potential);

    //let mut server = CapacityServer::<TDCapacityGraph, CCHPotential<FirstOutGraph<&[EdgeId], &[NodeId], &[Weight]>, FirstOutGraph<&[EdgeId], &[NodeId], &[Weight]>>>::new_with_potential(graph, cch_pot_data.forward_potential());
    //let mut server = CapacityServer::<TDCapacityGraph, ZeroPotential>::new(graph);

    /*generate_random_uniform_queries(num_nodes as u32, 100)
        .iter()
        .for_each(|query| {
            server.query(*query, true);
        });*/

    let slowdown_factor = 10.0;
    let num_queries = evaluate_potential_quality(&mut server, num_nodes, slowdown_factor);

    println!("Total number of queries until slowdown of {}: {}", slowdown_factor, num_queries);
}
