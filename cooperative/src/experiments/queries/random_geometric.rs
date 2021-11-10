use rand::{thread_rng, Rng};
use rand_distr::{Distribution, Geometric};

use rust_road_router::algo::dijkstra::{DefaultOps, DijkstraData, DijkstraInit, DijkstraRun};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Link, LinkIterable, NodeId};

use crate::experiments::queries::departure_distributions::DepartureDistribution;

const INV_AVERAGE_TRIP_LENGTH: f64 = 0.000025; // 1 / 40000, avg trip length is ~40 km

pub fn generate_random_geometric_queries<G: LinkIterable<Link>, D: DepartureDistribution>(
    graph: &G,
    num_queries: u32,
    mut departure_distribution: D,
) -> Vec<TDQuery<Timestamp>> {
    let mut rng = thread_rng();
    let distribution = Geometric::new(INV_AVERAGE_TRIP_LENGTH).unwrap();

    // init dijkstra context
    let mut data = DijkstraData::new(graph.num_nodes());

    let mut queries = (0..num_queries)
        .into_iter()
        .map(|idx| {
            let mut result: Option<TDQuery<Timestamp>> = None;

            while result.is_none() {
                // in (extremely rare) case a too high number gets selected
                let from = rng.gen_range(0..graph.num_nodes()) as NodeId;
                let distance = distribution.sample(&mut rng) as u32;

                let query = TDQuery::new(from, 0, 0);
                let mut ops = DefaultOps::default();
                let mut dijkstra = DijkstraRun::query(graph, &mut data, &mut ops, DijkstraInit::from_query(&query));

                while let Some(node) = dijkstra.next() {
                    // cancel as soon as the tentative distance exceeds the threshold
                    if *dijkstra.tentative_distance(node) > distance {
                        result = Some(TDQuery::new(from, node, departure_distribution.rand(&mut rng)));
                        break;
                    }
                }
            }

            if idx % 100 == 0 {
                println!("Finished {}/{} queries", idx, num_queries);
            }

            result.unwrap()
        })
        .collect::<Vec<TDQuery<Timestamp>>>();

    // sort queries by departure for a more realistic usage scenario
    queries.sort_by_key(|query| query.departure);

    queries
}
