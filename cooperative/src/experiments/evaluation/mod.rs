use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::Weight;

use crate::dijkstra::server::CapacityServerOps;

#[derive(Copy, Clone, Debug)]
pub struct EvaluationResult<W> {
    // average distance obtained by executing ALG on all query pairs (with updates)
    avg_dist_algo: W,
    // average distance of each query, considering the current traffic flow
    avg_dist_actual: W,

    // deviation between ALG and actual values
    avg_deviation: W,
    avg_deviation_rel: f64,

    // deviation quantiles between ALG and actual values ([min, 25%, mean, 75%, max])
    deviation_quantiles: [W; 5],
    deviation_quantiles_rel: [f64; 5],
}

impl EvaluationResult<Weight> {
    pub fn new(alg_result: &[Weight], actual_result: &[Weight]) -> Self {
        assert_eq!(alg_result.len(), actual_result.len(), "Results must have same length!");
        assert!(alg_result.len() > 0, "Queries must not be empty!");

        let num_queries = alg_result.len();

        let total_weight_algo = alg_result.iter().sum::<Weight>();
        let total_weight_actual = actual_result.iter().sum::<Weight>();

        // handle absolute deviations
        let mut abs_deviations = alg_result
            .iter()
            .zip(actual_result.iter())
            .map(|(&a, &b)| (a, b))
            .collect::<Vec<(Weight, Weight)>>();
        abs_deviations.sort_by_key(|k| abs_diff(*k));

        let abs_total_deviation = abs_deviations.iter().map(|a| abs_diff(*a)).sum::<Weight>();

        // handle relative deviations
        let mut rel_deviations = abs_deviations.iter().map(|a| rel_deviation(*a)).collect::<Vec<f64>>();
        rel_deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let rel_total_deviation = rel_deviations.iter().sum::<f64>();

        Self {
            avg_dist_algo: total_weight_algo / num_queries as Weight,
            avg_dist_actual: total_weight_actual / num_queries as Weight,

            avg_deviation: abs_total_deviation / num_queries as Weight,
            avg_deviation_rel: rel_total_deviation / num_queries as f64,

            deviation_quantiles: [
                abs_diff(*abs_deviations.first().unwrap()),
                abs_diff(*abs_deviations.get(num_queries / 4).unwrap()),
                abs_diff(*abs_deviations.get(num_queries / 2).unwrap()),
                abs_diff(*abs_deviations.get((3 * num_queries) / 4).unwrap()),
                abs_diff(*abs_deviations.last().unwrap()),
            ],
            deviation_quantiles_rel: [
                *rel_deviations.first().unwrap(),
                *rel_deviations.get(num_queries / 4).unwrap(),
                *rel_deviations.get(num_queries / 2).unwrap(),
                *rel_deviations.get((3 * num_queries) / 4).unwrap(),
                *rel_deviations.last().unwrap(),
            ],
        }
    }
}

fn abs_diff((a, b): (Weight, Weight)) -> Weight {
    if a > b {
        a - b
    } else {
        b - a
    }
}

fn rel_deviation((algo, actual): (Weight, Weight)) -> f64 {
    abs_diff((algo, actual)) as f64 / algo as f64
}

pub fn evaluate_queries(server: &mut impl CapacityServerOps, queries: &[TDQuery<Timestamp>]) -> EvaluationResult<Weight> {
    let distances_algo = queries
        .iter()
        .cloned()
        .map(|query| server.query(query, true).unwrap().distance)
        .collect::<Vec<Weight>>();

    let distances_actual = queries
        .iter()
        .cloned()
        .map(|query| server.query(query, false).unwrap().distance)
        .collect::<Vec<Weight>>();

    EvaluationResult::new(&distances_algo, &distances_actual)
}
