use std::error::Error;
use std::path::Path;

use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, Graph, NodeId, UnweightedFirstOutGraph, Weight};
use rust_road_router::datastr::rank_select_map::{BitVec, RankSelectMap};
use rust_road_router::io::{Load, Store};

use crate::graph::Capacity;
use crate::io::io_coordinates::load_coords;

/// Extract the largest strongly connected component of a given Graph.
/// This preprocessing step avoids invalid (s,t)-queries where t is not reachable from s.
/// The result will be written to the output directory
pub fn extract_largest_scc(graph_directory: &Path, out_directory: &Path) -> Result<(), Box<dyn Error>> {
    let first_out = Vec::load_from(graph_directory.join("first_out"))?;
    let head = Vec::load_from(graph_directory.join("head"))?;
    let geo_distance = Vec::<Weight>::load_from(graph_directory.join("geo_distance"))?;
    let travel_time = Vec::<Weight>::load_from(graph_directory.join("travel_time"))?;
    let capacity = Vec::<Capacity>::load_from(graph_directory.join("capacity"))?;
    let (longitude, latitude) = load_coords(graph_directory)?;
    let is_largest_scc = Vec::<u32>::load_from(&graph_directory.join("largest_scc"))?;

    // initialize RankSelectMap structure
    let mut bit_vec = BitVec::new(is_largest_scc.len());
    is_largest_scc
        .iter()
        .enumerate()
        .filter(|(_, &val)| val > 0)
        .for_each(|(idx, _)| bit_vec.set(idx));
    let rank_select_map = RankSelectMap::new(bit_vec);

    let graph = UnweightedFirstOutGraph::new(first_out.clone(), head.clone());

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
            new_longitude.push(longitude[node_id]);
            new_latitude.push(latitude[node_id]);

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
                new_distance.push(geo_distance[old_edge_id]);
                new_time.push(travel_time[old_edge_id]);
                new_capacity.push(capacity[old_edge_id]);
            });
        }
    }

    new_first_out.write_to(&out_directory.join("first_out"))?;
    new_head.write_to(&out_directory.join("head"))?;
    new_distance.write_to(&out_directory.join("geo_distance"))?;
    new_time.write_to(&out_directory.join("travel_time"))?;
    new_capacity.write_to(&out_directory.join("capacity"))?;
    new_longitude.write_to(&out_directory.join("longitude"))?;
    new_latitude.write_to(&out_directory.join("latitude"))?;

    Ok(())
}
