use rand::{Rng, thread_rng};

use rust_road_router::algo::{GenQuery, Query, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::graph::td_capacity_graph::MAX_BUCKETS;
use crate::experiments::generate_queries::departure_distributions::DepartureDistribution;

pub fn generate_random_uniform_queries(num_nodes: u32, num_queries: u32) -> Vec<Query> {
    let mut rng = thread_rng();

    (0..num_queries)
        .into_iter()
        .map(|_| {
            let (mut from, mut to) = (0, 0);
            while from == to {
                from = rng.gen_range(0..num_nodes);
                to = rng.gen_range(0..num_nodes);
            }
            Query::new(from, to, 0)
        })
        .collect::<Vec<Query>>()
}

pub fn generate_random_uniform_td_queries<D: DepartureDistribution>(
    num_nodes: u32,
    num_queries: u32,
    departure_distribution: D
) -> Vec<TDQuery<Timestamp>> {
    let mut rng = thread_rng();

    (0..num_queries)
        .into_iter()
        .map(|_| {
            let (mut from, mut to) = (0, 0);
            while from == to {
                from = rng.gen_range(0..num_nodes);
                to = rng.gen_range(0..num_nodes);
            }
            TDQuery::new(from, to, departure_distribution.rand(&mut rng))
        })
        .collect::<Vec<TDQuery<Timestamp>>>()
}