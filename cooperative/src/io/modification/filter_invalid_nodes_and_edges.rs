use crate::io::modification::CapacityGraphContainer;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, Graph, NodeId, UnweightedFirstOutGraph};
use rust_road_router::datastr::rank_select_map::{BitVec, RankSelectMap};
use std::collections::HashMap;

pub fn filter_invalid_nodes_and_edges(raw_data: &CapacityGraphContainer, is_valid_node: &Vec<bool>, is_valid_edge: &Vec<bool>) -> CapacityGraphContainer {
    // initialize RankSelectMap
    let mut bit_vec = BitVec::new(is_valid_node.len());
    is_valid_node.iter().enumerate().filter(|(_, &val)| val).for_each(|(idx, _)| bit_vec.set(idx));
    let rank_select_map = RankSelectMap::new(bit_vec);

    let graph = UnweightedFirstOutGraph::new(&raw_data.first_out, &raw_data.head);

    // allocate vectors
    let mut first_out = Vec::with_capacity(graph.num_nodes() + 1);
    first_out.push(0);
    let mut head = Vec::with_capacity(graph.num_arcs());
    let mut geo_distance = Vec::with_capacity(graph.num_arcs());
    let mut travel_time = Vec::with_capacity(graph.num_arcs());
    let mut max_capacity = Vec::with_capacity(graph.num_arcs());
    let mut longitude = Vec::with_capacity(graph.num_nodes());
    let mut latitude = Vec::with_capacity(graph.num_nodes());

    for node_id in 0..graph.num_nodes() {
        if rank_select_map.get(node_id).is_some() {
            // move coordinates
            longitude.push(raw_data.longitude[node_id]);
            latitude.push(raw_data.latitude[node_id]);

            // collect valid edges
            let mut neighbor_data = HashMap::<u32, (u32, u32, u32)>::new();
            graph
                .neighbor_edge_indices_usize(node_id as NodeId)
                .filter(|&edge_id| is_valid_edge[edge_id] && rank_select_map.get(graph.head()[edge_id] as usize).is_some())
                .for_each(|edge_id| {
                    let edge_head = rank_select_map.get(graph.head()[edge_id] as usize).unwrap() as NodeId;

                    if let Some((edge_capacity, edge_distance, edge_tt)) = neighbor_data.get_mut(&edge_head) {
                        // multi-edge found: sum up capacities, take tt/dist based on faster tt
                        *edge_capacity += raw_data.max_capacity[edge_id];

                        if *edge_tt > raw_data.travel_time[edge_id] {
                            *edge_tt = raw_data.travel_time[edge_id];
                            *edge_distance = raw_data.geo_distance[edge_id];
                        }
                    } else {
                        // new entry found
                        neighbor_data.insert(
                            edge_head,
                            (raw_data.max_capacity[edge_id], raw_data.geo_distance[edge_id], raw_data.travel_time[edge_id]),
                        );
                    }
                });

            // append data to new structs
            for (edge_head, (edge_capacity, edge_distance, edge_tt)) in neighbor_data {
                head.push(edge_head);
                max_capacity.push(edge_capacity);
                geo_distance.push(edge_distance);
                travel_time.push(edge_tt);
            }

            first_out.push(head.len() as EdgeId);
        }
    }

    CapacityGraphContainer {
        first_out,
        head,
        geo_distance,
        travel_time,
        max_capacity,
        latitude,
        longitude,
    }
}
