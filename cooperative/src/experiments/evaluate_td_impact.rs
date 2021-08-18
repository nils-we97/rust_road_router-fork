use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::{NodeId, Weight};

use crate::dijkstra::model::TDPathResult;
use crate::dijkstra::server::CapacityServerOps;
use crate::experiments::PathCompareResult;
use crate::graph::td_capacity_graph::TDCapacityGraph;

pub fn evaluate_time_dependent_impact<Pot>(
    server: &mut impl CapacityServerOps<TDCapacityGraph, TDPathResult, Pot>,
    td_server: &mut impl CapacityServerOps<TDCapacityGraph, TDPathResult, Pot>,
    queries: &[impl GenQuery<NodeId> + Clone],
) -> PathCompareResult {

    //TODO auf größerem Graphen Routen vergleichen!!

    // calculate paths
    let paths = calculate_paths(server, queries);
    println!("Finished baseline queries!");

    let td_paths = calculate_paths(td_server, queries);
    println!("Finished TD queries!");

    assert_eq!(paths.len(), td_paths.len(), "number of valid paths should be the same!");

    // compare paths, evaluate on `td_server`
    let distances = calculate_path_distances(td_server, &paths);
    let td_distances = calculate_path_distances(td_server, &td_paths);
    println!("Finished calculating total distances");

    PathCompareResult::new(paths.len() as u32, distances, td_distances)
}

fn calculate_paths<Pot>(
    server: &mut impl CapacityServerOps<TDCapacityGraph, TDPathResult, Pot>,
    queries: &[impl GenQuery<NodeId> + Clone],
) -> Vec<TDPathResult> {
    queries
        .iter()
        .cloned()
        .filter_map(|query| server
            .query(query, true)
            .map(|result| result.path)
        ).collect::<Vec<TDPathResult>>()
}

fn calculate_path_distances<Pot>(
    compare_server: &mut impl CapacityServerOps<TDCapacityGraph, TDPathResult, Pot>,
    paths: &Vec<TDPathResult>
) -> Weight {

    paths
        .iter()
        .map(|path| compare_server.path_distance(path))
        .sum()
}