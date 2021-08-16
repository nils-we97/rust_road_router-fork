use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::{NodeId, Weight};

use crate::dijkstra::model::{PathResult, TDPathResult};
use crate::dijkstra::server::CapacityServerOps;
use crate::experiments::PathCompareResult;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::td_capacity_graph::TDCapacityGraph;

pub fn evaluate_time_dependent_impact(
    server: &mut impl CapacityServerOps<CapacityGraph, PathResult>,
    td_server: &mut impl CapacityServerOps<TDCapacityGraph, TDPathResult>,
    queries: &[impl GenQuery<NodeId> + Clone],
) -> PathCompareResult {
    let (count, sum_distances) = calculate_distances(server, queries);

    println!("finished queries (1/2)");

    let (td_count, sum_td_distances) = calculate_distances(td_server, queries);

    println!("finished td-queries (2/2)");

    assert_eq!(count, td_count, "number of paths should be the same!");

    PathCompareResult::new(count as u32, sum_distances, sum_td_distances)
}

fn calculate_distances<G, P>(
    server: &mut impl CapacityServerOps<G, P>,
    queries: &[impl GenQuery<NodeId> + Clone],
) -> (usize, Weight) {
    let paths = queries
        .iter()
        .cloned()
        .filter_map(|query| server
            .query(query, true)
            .map(|result| result.path)
        ).collect::<Vec<P>>();

    let distances = paths
        .iter()
        .map(|path| server.path_distance(path))
        .collect::<Vec<Weight>>();

    (distances.len(), distances.iter().sum())
}