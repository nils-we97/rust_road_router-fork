use std::cmp::max;

use cooperative::graph::capacity_graph::CapacityGraph;
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::graph::Capacity;
use rust_road_router::datastr::graph::{NodeId, Weight};

pub struct CapacityEdge {
    pub from: NodeId,
    pub to: NodeId,
    pub distance: Weight,
    pub travel_time: Weight,
    pub capacity: Capacity,
}

impl CapacityEdge {
    pub fn new(from: NodeId, to: NodeId, distance: Weight, travel_time: Weight, capacity: Capacity) -> Self {
        Self {
            from,
            to,
            distance,
            travel_time,
            capacity,
        }
    }
}

/// Creates a dummy graph from a given (unordered) edge list.
pub fn create_graph(num_buckets: u32, mut edges: Vec<CapacityEdge>) -> CapacityGraph {
    assert!(!edges.is_empty(), "graph must not be empty!");

    // bring edges in sorted order
    edges.sort_by_key(|a| (a.from, a.to));

    let max_node_id = edges.iter().map(|edge| max(edge.from, edge.to)).max().unwrap();

    let mut head = Vec::with_capacity(edges.len());
    let mut distance = Vec::with_capacity(edges.len());
    let mut freeflow_time = Vec::with_capacity(edges.len());
    let mut max_capacity = Vec::with_capacity(edges.len());

    let mut degree = vec![0u32; max_node_id as usize];
    let mut first_out = vec![0];

    edges.iter().for_each(|edge| {
        degree[edge.from as usize] += 1;
        head.push(edge.to);
        distance.push(edge.distance);
        freeflow_time.push(edge.travel_time);
        max_capacity.push(edge.capacity);
    });

    degree.iter().for_each(|&deg| first_out.push(*first_out.last().unwrap() + deg));
    CapacityGraph::new(num_buckets, first_out, head, distance, freeflow_time, max_capacity, bpr_speed_function)
}

#[test]
fn build_dummy_graph() {
    let edges = vec![
        CapacityEdge::new(0, 1, 10, 10, 10),
        CapacityEdge::new(1, 2, 10, 10, 10),
        CapacityEdge::new(2, 3, 10, 10, 10),
    ];

    let graph = create_graph(1, edges);
    dbg!(&graph);

    assert_eq!(graph.first_out(), &[0, 1, 2, 3]);
    assert_eq!(graph.head(), &[1, 2, 3]);
}
