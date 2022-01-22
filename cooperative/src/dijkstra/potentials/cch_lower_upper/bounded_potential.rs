use crate::dijkstra::potentials::cch_lower_upper::elimination_tree_server::CorridorEliminationTreeServer;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

#[derive(Clone, Debug)]
pub struct BoundedLowerUpperPotentialContext {
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<(Weight, Weight)>>,
    forward_distances: TimestampedVector<(Weight, Weight)>,
    backward_distances: TimestampedVector<(Weight, Weight)>,
    target_bounds: Option<(Weight, Weight)>,
    num_pot_computations: usize,
}

impl BoundedLowerUpperPotentialContext {
    pub fn new(num_nodes: usize) -> Self {
        Self {
            stack: Vec::new(),
            potentials: TimestampedVector::new(num_nodes),
            forward_distances: TimestampedVector::new(num_nodes),
            backward_distances: TimestampedVector::new(num_nodes),
            target_bounds: None,
            num_pot_computations: 0,
        }
    }
}

pub struct BoundedLowerUpperPotential<'a, CCH> {
    cch: &'a CCH,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<(Weight, Weight)>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<(Weight, Weight)>,
    context: &'a mut BoundedLowerUpperPotentialContext,
}

impl<'a, CCH: CCHT> BoundedLowerUpperPotential<'a, CCH> {
    pub fn prepare(
        cch: &'a CCH,
        forward_cch_weights: &'a Vec<(Weight, Weight)>,
        backward_cch_weights: &'a Vec<(Weight, Weight)>,
        context: &'a mut BoundedLowerUpperPotentialContext,
    ) -> Self {
        let forward_cch_graph = UnweightedFirstOutGraph::new(cch.forward_first_out(), cch.forward_head());
        let backward_cch_graph = UnweightedFirstOutGraph::new(cch.backward_first_out(), cch.backward_head());

        Self {
            cch,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            context,
        }
    }

    pub fn init(&mut self, source: u32, target: u32) -> Option<(Weight, Weight)> {
        self.context.potentials.reset();
        self.context.num_pot_computations = 0;

        // 1. interval query to determine bounds at target node
        self.context.target_bounds = CorridorEliminationTreeServer::query(
            self.cch,
            &self.forward_cch_graph,
            self.forward_cch_weights,
            &self.backward_cch_graph,
            self.backward_cch_weights,
            &mut self.context.forward_distances,
            &mut self.context.backward_distances,
            source,
            target,
        );

        // forward search space is already initialized -> nothing to do here :)
        /*if self.context.target_bounds.is_some() {
            // 2. initialize forward-upward search space with distances from interval query
            let source = self.cch.node_order().rank(source);
            let query_forward_distances = self.elimination_tree_server.forward_distances();

            // updating the distances is straightforward and we do not have to relax any edges!
            let mut cur_node = Some(source);
            while let Some(node) = cur_node {
                cur_node = self.cch.elimination_tree()[node as usize].value();
                self.forward_distances[node as usize] = query_forward_distances[node as usize];
            }
        }*/

        self.context.target_bounds
    }

    pub fn potential_bounds(&mut self, node: NodeId) -> Option<(Weight, Weight)> {
        let rank = self.cch.node_order().rank(node);
        if let Some((_, target_upper)) = self.context.target_bounds {
            // upward search until a node with existing distance to target is found
            let mut cur_node = rank;
            while self.context.potentials[cur_node as usize].value().is_none() {
                self.context.num_pot_computations += 1;
                self.context.stack.push(cur_node);
                if let Some(parent) = self.cch.elimination_tree()[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // propagate the result back to the original start node, do some additional pruning
            while let Some(current_node) = self.context.stack.pop() {
                let (mut dist_lower, mut dist_upper) = self.context.forward_distances[current_node as usize];

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, current_node) {
                    let (edge_weight_lower, edge_weight_upper) = self.backward_cch_weights[edge as usize];
                    let (next_potential_lower, next_potential_upper) = self.context.potentials[next_node as usize].value().unwrap();

                    dist_lower = min(dist_lower, edge_weight_lower + next_potential_lower);
                    dist_upper = min(dist_upper, edge_weight_upper + next_potential_upper);
                }

                // pruning: ignore node if the lower bound already exceeds the known upper bound to the target
                if dist_lower > target_upper {
                    dist_lower = INFINITY;
                    dist_upper = INFINITY;
                }

                self.context.potentials[current_node as usize] = InRangeOption::new(Some((dist_lower, dist_upper)));
            }

            self.context.potentials[rank as usize].value().filter(|&(lower, _)| lower < INFINITY)
        } else {
            None
        }
    }
}
