use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::dijkstra::model::PathResult;
use crate::dijkstra::server::CapacityServerOps;
use crate::experiments::PathCompareResult;

pub fn compare_static_cooperative<Pot>(server: &mut impl CapacityServerOps<Pot>, queries: &[TDQuery<Timestamp>]) -> PathCompareResult {
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
        .collect::<Vec<PathResult>>();

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
        .collect::<Vec<PathResult>>();

    assert_eq!(static_paths.len(), dynamic_paths.len(), "number of paths should be the same!");

    let total_dist_static = static_paths.iter().map(|path| server.path_distance(path)).sum();
    let total_dist_dynamic = dynamic_paths.iter().map(|path| server.path_distance(path)).sum();

    PathCompareResult::new(static_paths.len() as u32, total_dist_static, total_dist_dynamic)
}
