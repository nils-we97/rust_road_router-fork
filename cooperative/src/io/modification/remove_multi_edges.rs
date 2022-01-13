use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::traffic_functions::bpr_traffic_function;
use rust_road_router::datastr::graph::{EdgeIdGraph, EdgeIdT, Graph, NodeId};

pub fn remove_multi_edges(graph: &CapacityGraph) -> CapacityGraph {
    let num_buckets = graph.num_buckets();
    let mut first_out = vec![0];
    let mut head = Vec::new();

    let mut distance = Vec::new();
    let mut free_flow_travel_time = Vec::new();
    let mut max_capacity = Vec::new();

    for node in 0..graph.num_nodes() as u32 {
        let mut neighbors = graph
            .neighbor_edge_indices_usize(node)
            .into_iter()
            .map(|e| graph.head()[e])
            .collect::<Vec<NodeId>>();

        neighbors.sort();
        neighbors.dedup();

        let num_neighbors = neighbors.len();

        for neighbor in neighbors {
            graph
                .edge_indices(node, neighbor)
                .max_by_key(|&EdgeIdT(e)| graph.max_capacity()[e as usize])
                .map(|EdgeIdT(edge)| {
                    let edge = edge as usize;

                    head.push(graph.head()[edge]);
                    distance.push(graph.distance()[edge]);
                    free_flow_travel_time.push(graph.free_flow_time()[edge]);
                    max_capacity.push(graph.max_capacity()[edge]);
                });
        }

        first_out.push(*first_out.last().unwrap() + num_neighbors as u32);
    }

    println!("Reduced to {} nodes, {} edges", first_out.len(), head.len());
    println!("Before: {} nodes, {} edges", graph.first_out().len(), graph.head().len());

    CapacityGraph::new(
        num_buckets,
        first_out,
        head,
        distance,
        free_flow_travel_time,
        max_capacity,
        bpr_traffic_function,
    )
}
