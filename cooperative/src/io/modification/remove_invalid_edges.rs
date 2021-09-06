use crate::io::modification::extract_scc::extract_largest_scc;
use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight};
use rust_road_router::io::{Load, Store};
use std::collections::VecDeque;
use std::error::Error;
use std::path::Path;

/// Graph preprocessing: remove invalid edges (e.g. ferry connections) and delete 0-degree-nodes afterwards
/// This step is needed in order to avoid exceptions when creating TTF profiles
/// where edges with a travel time > 24 hours are not working
pub fn remove_invalid_edges(graph_directory: &Path, scc_representant: NodeId) -> Result<(), Box<dyn Error>> {
    let first_out = Vec::<EdgeId>::load_from(graph_directory.join("first_out"))?;
    let head = Vec::<NodeId>::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::<Weight>::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::<Weight>::load_from(graph_directory.join("travel_time"))?;
    let num_edges = head.len();

    // mark invalid edges (travel time > 1 hour or distance > 20km)
    let is_edge_valid = (0..num_edges)
        .into_iter()
        .map(|edge_id| travel_time[edge_id] <= 3600 && geo_distance[edge_id] <= 20000)
        .collect::<Vec<bool>>();

    // mark reachable nodes (bfs from `largest_scc_node` over valid edges only)
    let reachable_nodes = get_reachable_nodes(&first_out, &head, &is_edge_valid, scc_representant)
        .iter()
        .map(|&val| val as u32)
        .collect::<Vec<u32>>();

    // treat all reachable nodes like the largest scc, run scc extraction as subroutine
    // I/O could easily be reduced, but that's not necessary for now
    reachable_nodes.write_to(&graph_directory.join("largest_scc"))?;
    println!("Finished writing list of reachable nodes!");
    extract_largest_scc(graph_directory, graph_directory)
}

fn get_reachable_nodes(first_out: &Vec<EdgeId>, head: &Vec<NodeId>, is_edge_valid: &Vec<bool>, init_node: NodeId) -> Vec<bool> {
    let mut is_vertex_reachable = vec![false; first_out.len() - 1];
    let mut queue = VecDeque::new();

    queue.push_back(init_node);
    is_vertex_reachable[init_node as usize] = true;

    while let Some(current) = queue.pop_front() {
        (first_out[current as usize]..first_out[(current + 1) as usize])
            .into_iter()
            .filter(|&edge_id| is_edge_valid[edge_id as usize])
            .for_each(|edge_id| {
                let target = head[edge_id as usize];
                if !is_vertex_reachable[target as usize] {
                    is_vertex_reachable[target as usize] = true;
                    queue.push_back(target);
                }
            });
    }

    let count = (0..first_out.len() - 1).into_iter().filter(|&idx| is_vertex_reachable[idx]).count();
    println!("Number of reachable nodes: {} of {}", count, first_out.len() - 1);

    is_vertex_reachable
}
