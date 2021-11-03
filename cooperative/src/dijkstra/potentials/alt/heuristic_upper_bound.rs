use crate::dijkstra::potentials::alt::HeuristicUpperBoundALTPotentialContext;
use rust_road_router::algo::a_star::Potential;
use rust_road_router::algo::alt::ALTPotential;
use rust_road_router::datastr::graph::{NodeId, Weight};
use std::cmp::max;

pub struct HeuristicUpperBoundALTPotential<'a> {
    lower_bound_potential: ALTPotential<'a>,
}

impl<'a> HeuristicUpperBoundALTPotential<'a> {
    pub fn new(context: &'a HeuristicUpperBoundALTPotentialContext) -> Self {
        Self {
            lower_bound_potential: context.pot_data.forward_potential(),
        }
    }

    pub fn potential_bounds(&mut self, node: NodeId) -> Option<(Weight, Weight)> {
        // current heuristic: upper value = lower + 10%, but at least 30 minutes
        self.potential(node).map(|lower| (lower, lower + max(lower / 10, 30 * 60 * 1000)))
    }
}

impl<'a> Potential for HeuristicUpperBoundALTPotential<'a> {
    fn init(&mut self, target: NodeId) {
        self.lower_bound_potential.init(target);
    }

    fn potential(&mut self, node: NodeId) -> Option<Weight> {
        self.lower_bound_potential.potential(node)
    }
}
