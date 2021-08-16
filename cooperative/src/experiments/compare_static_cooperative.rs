use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::NodeId;

use crate::dijkstra::server::CapacityServerOps;
use crate::experiments::PathCompareResult;

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
        })
        .collect::<Vec<P>>();

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
        })
        .collect::<Vec<P>>();

    assert_eq!(static_paths.len(), dynamic_paths.len(), "number of paths should be the same!");

    let total_dist_static = static_paths.iter().map(|path| server.path_distance(path)).sum();
    let total_dist_dynamic = dynamic_paths.iter().map(|path| server.path_distance(path)).sum();

    PathCompareResult::new(
        static_paths.len() as u32,
        total_dist_static,
        total_dist_dynamic,
    )
}
