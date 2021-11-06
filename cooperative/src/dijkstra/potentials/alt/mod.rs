use rust_road_router::algo::alt::ALTPotData;
use rust_road_router::datastr::graph::{EdgeId, FirstOutGraph, NodeId, Weight};
use rust_road_router::experiments;

pub mod heuristic_upper_bound;

pub struct HeuristicUpperBoundALTPotentialContext {
    lower_bound_weights: Vec<Weight>,
    pot_data: ALTPotData,
}

impl HeuristicUpperBoundALTPotentialContext {
    pub fn init(first_out: Vec<EdgeId>, head: Vec<NodeId>, travel_times: &Vec<Vec<Weight>>) -> Self {
        let weights = travel_times.iter().map(|ttf| ttf.iter().min().cloned().unwrap()).collect::<Vec<Weight>>();

        let graph = FirstOutGraph::new(first_out, head, weights.clone());
        let pot_data = ALTPotData::new_with_avoid(&graph, 4, &mut experiments::rng(Default::default()));

        Self {
            lower_bound_weights: weights,
            pot_data,
        }
    }
}
