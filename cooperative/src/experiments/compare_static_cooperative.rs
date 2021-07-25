use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::NodeId;

use crate::dijkstra::server::CapacityServerOps;

#[derive(Copy, Clone, Debug)]
pub struct PathCompareResult {
    pub num_queries: u32,
    pub total_distance_static: u32,
    pub total_distance_cooperative: u32,
    pub improvement: f64
}

pub fn compare_static_cooperative<G, P>(
    server: &mut impl CapacityServerOps<G, P>,
    queries: &[impl GenQuery<NodeId> + Clone]
) -> PathCompareResult {
    let static_paths = queries
        .iter()
        .cloned()
        .filter_map(|query| {
            let query_result = server.query(query, false);

            if query_result.is_some() {
                Some(query_result.unwrap().path)
            } else {
                None
            }
        }).collect::<Vec<P>>();

    let dynamic_paths = queries
        .iter()
        .cloned()
        .filter_map(|query| {
            let query_result = server.query(query, true);

            if query_result.is_some() {
                Some(query_result.unwrap().path)
            } else {
                None
            }
        }).collect::<Vec<P>>();

    assert_eq!(static_paths.len(), dynamic_paths.len(), "number of paths should be the same!");

    let total_dist_static = static_paths
        .iter()
        .map(|path| server.path_distance(path))
        .sum();

    let total_dist_dynamic = dynamic_paths
        .iter()
        .map(|path| server.path_distance(path))
        .sum();

    PathCompareResult {
        num_queries: static_paths.len() as u32,
        total_distance_static: total_dist_static,
        total_distance_cooperative: total_dist_dynamic,
        improvement: total_dist_static as f64 / total_dist_dynamic as f64
    }
}