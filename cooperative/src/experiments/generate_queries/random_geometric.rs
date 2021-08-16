use rand::{thread_rng, Rng};
use rand_distr::{Distribution, Geometric};

use rust_road_router::algo::dijkstra::{DefaultOpsWithLinkPath, DijkstraData, DijkstraRun};
use rust_road_router::algo::{GenQuery, Query};
use rust_road_router::datastr::graph::{EdgeId, FirstOutGraph, Graph, NodeId, Weight};

//const INV_AVERAGE_TRIP_LENGTH: f64 = 0.000025; // 1 / 40000, avg trip length is ~40 km

pub fn generate_random_geometric_queries(
    graph: FirstOutGraph<Vec<EdgeId>, Vec<NodeId>, Vec<Weight>>,
    num_queries: u32,
    inv_average_trip_length: f64,
) -> Vec<Query> {
    let mut rng = thread_rng();
    let distribution = Geometric::new(inv_average_trip_length).unwrap();

    // init dijkstra context
    let mut data = DijkstraData::new(graph.num_nodes());

    (0..num_queries)
        .into_iter()
        .map(|_| {
            let mut result: Option<Query> = None;

            while result.is_none() {
                // in (extremely rare) case a too high number gets selected
                let from = rng.gen_range(0..graph.num_nodes()) as NodeId;
                let distance = distribution.sample(&mut rng) as u32;

                let query = Query::new(from, 0, 0);
                let mut ops = DefaultOpsWithLinkPath::default();
                let mut dijkstra = DijkstraRun::query(&graph, &mut data, &mut ops, query);

                while let Some(node) = dijkstra.next() {
                    // cancel as soon as the tentative distance exceeds the threshold
                    if *dijkstra.tentative_distance(node) > distance {
                        result = Some(Query::new(from, node, 0));
                        break;
                    }
                }
            }

            result.unwrap()
        })
        .collect::<Vec<Query>>()
}
