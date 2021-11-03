use rand::{thread_rng, Rng};

use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::experiments::queries::departure_distributions::DepartureDistribution;

pub fn generate_random_uniform_queries<D: DepartureDistribution>(num_nodes: u32, num_queries: u32, mut departure_distribution: D) -> Vec<TDQuery<Timestamp>> {
    let mut rng = thread_rng();

    let mut queries = (0..num_queries)
        .into_iter()
        .map(|_| {
            let (mut from, mut to) = (0, 0);
            while from == to {
                from = rng.gen_range(0..num_nodes);
                to = rng.gen_range(0..num_nodes);
            }
            TDQuery::new(from, to, departure_distribution.rand(&mut rng))
        })
        .collect::<Vec<TDQuery<Timestamp>>>();

    // sort queries by departure for a more realistic usage scenario
    queries.sort_by_key(|query| query.departure);

    queries
}
