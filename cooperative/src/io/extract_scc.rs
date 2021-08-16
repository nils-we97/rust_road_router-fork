use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, Graph, NodeId, UnweightedFirstOutGraph};
use rust_road_router::datastr::rank_select_map::{BitVec, RankSelectMap};
use rust_road_router::io::Load;

use crate::io::io_raw_graph::{load_capacity_graph_raw, store_capacity_graph_raw, RawCapacityGraphContainer};

/// Extract the largest strongly connected component of a given Graph.
/// This preprocessing step avoids invalid (s,t)-queries where t is not reachable from s.
pub fn extract_largest_scc(graph_directory: &Path, out_directory: &Path) -> Result<(), Box<dyn Error>> {
    let graph_container = load_capacity_graph_raw(&graph_directory)?;
    let is_largest_scc: Vec<u32> = Vec::load_from(&graph_directory.join("largest_scc"))?;

    // initialize RankSelectMap structure
    let mut bit_vec = BitVec::new(is_largest_scc.len());
    is_largest_scc
        .iter()
        .enumerate()
        .filter(|(_, &val)| val > 0)
        .for_each(|(idx, _)| bit_vec.set(idx));
    let rank_select_map = RankSelectMap::new(bit_vec);

    let graph = UnweightedFirstOutGraph::new(graph_container.first_out.clone(), graph_container.head.clone());

    let mut new_first_out = Vec::with_capacity(graph.num_nodes() + 1);
    let mut new_head = Vec::with_capacity(graph.num_arcs());
    let mut new_distance = Vec::with_capacity(graph.num_arcs());
    let mut new_time = Vec::with_capacity(graph.num_arcs());
    let mut new_capacity = Vec::with_capacity(graph.num_arcs());
    let mut new_longitude = Vec::with_capacity(graph.num_nodes());
    let mut new_latitude = Vec::with_capacity(graph.num_nodes());

    new_first_out.push(0);
    for node_id in 0..graph.num_nodes() {
        if rank_select_map.get(node_id).is_some() {
            // move coordinates
            new_longitude.push(graph_container.longitude[node_id]);
            new_latitude.push(graph_container.latitude[node_id]);

            // move edge-related information
            let remaining_neighbors = graph
                .neighbor_edge_indices(node_id as NodeId)
                .filter_map(|edge_id| {
                    let edge_id = edge_id as usize;
                    rank_select_map.get(graph.head()[edge_id] as usize).map(|target| (edge_id, target as NodeId))
                })
                .collect::<Vec<(usize, NodeId)>>();

            let prefix_sum = new_first_out.last().unwrap() + remaining_neighbors.len() as EdgeId;
            new_first_out.push(prefix_sum);

            remaining_neighbors.iter().for_each(|&(old_edge_id, new_target_vertex_id)| {
                new_head.push(new_target_vertex_id);
                new_distance.push(graph_container.geo_distance[old_edge_id]);
                new_time.push(graph_container.travel_time[old_edge_id]);
                new_capacity.push(graph_container.capacity[old_edge_id]);
            });
        }
    }

    let container = RawCapacityGraphContainer {
        first_out: new_first_out,
        head: new_head,
        geo_distance: new_distance,
        travel_time: new_time,
        capacity: new_capacity,
        longitude: new_longitude,
        latitude: new_latitude,
    };

    store_capacity_graph_raw(&container, &out_directory)
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
