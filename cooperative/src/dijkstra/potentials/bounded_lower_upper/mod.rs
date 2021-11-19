use crate::dijkstra::elimination_tree::corridor_intervals::server::CorridorEliminationTreeServer;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub struct BoundedLowerUpperPotential<'a, CCH> {
    cch: &'a CCH,
    elimination_tree_server: CorridorEliminationTreeServer<'a, CCH>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<(Weight, Weight)>>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: Vec<(Weight, Weight)>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: Vec<(Weight, Weight)>,
    forward_distances: TimestampedVector<(Weight, Weight)>,
    num_pot_computations: usize,
    target_bounds: Option<(Weight, Weight)>,
}

impl<'a, CCH: CCHT> BoundedLowerUpperPotential<'a, CCH> {
    pub fn new(cch: &'a CCH, forward_cch_weights: Vec<(Weight, Weight)>, backward_cch_weights: Vec<(Weight, Weight)>) -> Self {
        let forward_cch_graph = UnweightedFirstOutGraph::new(cch.forward_first_out(), cch.forward_head());
        let backward_cch_graph = UnweightedFirstOutGraph::new(cch.backward_first_out(), cch.backward_head());
        let n = forward_cch_graph.num_nodes();

        let elimination_tree_server = CorridorEliminationTreeServer::new(
            cch,
            forward_cch_graph.clone(),
            forward_cch_weights.clone(),
            backward_cch_graph.clone(),
            backward_cch_weights.clone(),
        );

        Self {
            elimination_tree_server,
            cch,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_distances: TimestampedVector::new(n),
            num_pot_computations: 0,
            target_bounds: None,
        }
    }

    pub fn init(&mut self, source: u32, target: u32) -> Option<(Weight, Weight)> {
        self.potentials.reset();
        self.forward_distances.reset();
        self.num_pot_computations = 0;

        // 1. interval query to determine bounds at target node
        self.target_bounds = self.elimination_tree_server.query(source, target);

        if self.target_bounds.is_some() {
            // 2. initialize forward-upward search space with distances from interval query
            let source = self.cch.node_order().rank(source);
            let query_forward_distances = self.elimination_tree_server.forward_distances();

            // updating the distances is straightforward and we do not have to relax any edges!
            let mut cur_node = Some(source);
            while let Some(node) = cur_node {
                cur_node = self.cch.elimination_tree()[node as usize].value();
                self.forward_distances[node as usize] = query_forward_distances[node as usize];
            }
        }

        self.target_bounds
    }

    pub fn potential_bounds(&mut self, rank: NodeId) -> Option<(Weight, Weight)> {
        if let Some((_, target_upper)) = self.target_bounds {
            // upward search until a node with existing distance to target is found
            let mut cur_node = rank;
            while self.potentials[cur_node as usize].value().is_none() {
                self.num_pot_computations += 1;
                self.stack.push(cur_node);
                if let Some(parent) = self.cch.elimination_tree()[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // propagate the result back to the original start node, do some additional pruning
            while let Some(current_node) = self.stack.pop() {
                let (mut dist_lower, mut dist_upper) = self.forward_distances[current_node as usize];

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, current_node) {
                    let (edge_weight_lower, edge_weight_upper) = self.backward_cch_weights[edge as usize];
                    let (next_potential_lower, next_potential_upper) = self.potentials[next_node as usize].value().unwrap();

                    dist_lower = min(dist_lower, edge_weight_lower + next_potential_lower);
                    dist_upper = min(dist_upper, edge_weight_upper + next_potential_upper);
                }

                // pruning: ignore node if the lower bound already exceeds the known upper bound to the target
                let (pot_lower, pot_upper) = if dist_lower < target_upper {
                    (dist_lower, dist_upper)
                } else {
                    (INFINITY, INFINITY)
                };

                self.potentials[current_node as usize] = InRangeOption::new(Some((pot_lower, pot_upper)));
                //self.potentials[current_node as usize] = InRangeOption::new(Some((0, target_upper)));
            }

            self.potentials[rank as usize].value().filter(|&(lower, _)| lower < INFINITY)
        } else {
            None
        }
    }
}
