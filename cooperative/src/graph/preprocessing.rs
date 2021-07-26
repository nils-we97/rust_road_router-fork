use std::collections::HashMap;

use rust_road_router::datastr::graph::{EdgeId, Graph, INFINITY, NodeId, RandomLinkAccessGraph};
use rust_road_router::report::measure;

use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::traffic_functions::time_functions::bpr_traffic_function;

/// Extract the largest strongly connected component of a given Graph.
/// This preprocessing step avoids invalid (s,t)-queries where t is not reachable from s.
/// The implementation is an iterative version of the textbook algorithm provided by the Algo2 lecture
pub fn largest_scc(graph: &CapacityGraph, lon: &Vec<f32>, lat: &Vec<f32>) -> (CapacityGraph, Vec<f32>, Vec<f32>) {
    let (component, time) = measure(|| dfs(graph));
    println!("SCCs computed in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // find the largest component
    let mut map = HashMap::new();

    for i in 0..graph.num_nodes() {
        let prev_hits = if map.contains_key(&component[i]) { map[&component[i]] } else { 0u32 };
        map.insert(component[i], prev_hits + 1);
    }

    let (&largest_scc, _) = map.iter().max_by_key(|(_, &v)| v).unwrap();
    println!("Largest SCC contains {} of {} nodes", map[&largest_scc], graph.num_nodes());

    let ((ret, lon, lat), time) = measure(||
        induced_subgraph(&graph, lon, lat, &component, largest_scc)
    );
    println!(
        "Subgraph created in {} ms. It contains {} vertices and {} edges",
        time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        ret.num_nodes(),
        ret.num_arcs()
    );

    (ret, lon, lat)
}

/// Shrink the graph by removing all degree 2 vertices.
/// (1) For all vertices v with out_edge w and in_edge u != w:
///    insert {u, w} and remove v
/// (2) For all vertices v with out_edges {u, w}:
///     if {u, v}, {v, u}, {v, w}, {w, v} all in E, then insert {u, w}, {w, u} and remove v

pub fn remove_deg2_nodes() {
    todo!()
    // can be useful when working with shortcuts! this hopefully keeps the graph sufficiently small

    // 1. sort vertices by degree, store in priority list
    // 2. for all vertices with degree 1: apply rule (1) recursively
    // 3. for all vertices with degree 2: apply rule (2) recursively
    // 4. adjust vertex/edge ids/coordinates, build new graph
}

fn dfs(graph: &CapacityGraph) -> Vec<NodeId> {
    // dfs structures
    let mut visited = vec![false; graph.num_nodes()];
    let mut dfs_stack = Vec::new();
    let mut dfs_num = vec![INFINITY; graph.num_nodes()];
    let mut dfs_pos = 1;

    // scc structures
    let mut component = vec![(graph.num_nodes() + 1) as NodeId; graph.num_nodes()];
    let mut reps = Vec::new();
    let mut node_stack = Vec::new();

    let mut num_visited_vertices = 0;

    for i in 0..graph.num_nodes() {
        let node = i as NodeId;

        if !visited[i] {
            visited[i] = true;
            num_visited_vertices += 1;

            //root (node)
            dfs_num[i] = dfs_pos;
            dfs_pos += 1;
            reps.push(node);
            node_stack.push(node);
        }

        dfs_stack.push((node, node));

        while !dfs_stack.is_empty() {
            let (_, current) = dfs_stack.pop().unwrap();

            for edge_id in graph.neighbor_edge_indices(current) {
                let target = graph.head(edge_id);

                if visited[target as usize] {
                    // traverse non tree edge (current, target)
                    if node_stack.contains(&target) {
                        assert!(!reps.is_empty());
                        while dfs_num[target as usize] < dfs_num[*reps.last().unwrap() as usize] {
                            reps.pop();
                        }
                    }
                } else {
                    // traverse tree edge (current, target)
                    reps.push(target);
                    node_stack.push(target);
                    dfs_num[target as usize] = dfs_pos;
                    dfs_pos += 1;

                    visited[target as usize] = true;
                    num_visited_vertices += 1;
                    dfs_stack.push((current, target));

                    if num_visited_vertices % 1000 == 0 {
                        println!("Inspected {} vertices", num_visited_vertices);
                    }
                }
            }
            // backtrack (pred, current)
            if !reps.is_empty() && current == *reps.last().unwrap() {
                reps.pop();

                let mut w = node_stack.pop().unwrap();
                component[w as usize] = current;

                while w != current {
                    w = node_stack.pop().unwrap();
                    component[w as usize] = current;
                }
            }
        }
    }

    component
}

/// build subgraph induced by the largest SCC
/// only consider edges (a, b) where component[a] = component[b] = max_component
fn induced_subgraph(
    graph: &CapacityGraph,
    lon: &Vec<f32>,
    lat: &Vec<f32>,
    component: &Vec<NodeId>,
    max_component: NodeId) -> (CapacityGraph, Vec<f32>, Vec<f32>) {
    let mut first_out = Vec::with_capacity(graph.num_nodes() + 1);
    let mut head = Vec::with_capacity(graph.num_arcs());
    let mut weight = Vec::with_capacity(graph.num_arcs());
    let mut capacity = Vec::with_capacity(graph.num_arcs());

    let mut new_lon = Vec::with_capacity(graph.num_nodes());
    let mut new_lat = Vec::with_capacity(graph.num_nodes());

    first_out.push(0);

    // relabel vertices
    let invalid_id = graph.num_nodes() as NodeId;
    let mut perm = vec![invalid_id; graph.num_nodes()];
    let mut current_node_id = 0;


    for node in 0..graph.num_nodes() {
        if component[node] == max_component {
            perm[node] = current_node_id;
            current_node_id += 1;
            new_lon.push(lon[node]);
            new_lat.push(lat[node]);
        }
    }

    for node in 0..graph.num_nodes() {
        if perm[node] < invalid_id {
            let filtered_edges = graph
                .neighbor_edge_indices(node as NodeId)
                .filter(|&edge_id| perm[graph.head(edge_id) as usize] < invalid_id)
                .collect::<Vec<EdgeId>>();

            let prev_value = *first_out.last().unwrap();
            first_out.push(prev_value + filtered_edges.len() as EdgeId);

            filtered_edges.iter().for_each(|&edge_id| {
                head.push(perm[graph.head(edge_id) as usize]);
                weight.push(graph.weight(edge_id));
                capacity.push(graph.capacity(edge_id));
            });
        }
    }

    let graph = CapacityGraph::new(
        first_out,
        head,
        weight,
        capacity,
        bpr_traffic_function,
    );

    assert_eq!(graph.num_nodes(), new_lon.len());
    assert_eq!(graph.num_nodes(), new_lat.len());

    (graph, new_lon, new_lat)
}