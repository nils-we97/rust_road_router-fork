use crate::dijkstra::potentials::cch_lower_upper::elimination_tree_server::CorridorEliminationTreeWalk;
use rust_road_router::algo::a_star::Potential;
use rust_road_router::algo::customizable_contraction_hierarchy::CCHT;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub struct CCHLowerUpperPotential<'a, CCH> {
    cch: &'a CCH,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<(Weight, Weight)>>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: Vec<(Weight, Weight)>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: Vec<(Weight, Weight)>,
    backward_distances: TimestampedVector<(Weight, Weight)>,
    num_pot_computations: usize,
}

impl<'a, CCH: CCHT> CCHLowerUpperPotential<'a, CCH> {
    pub fn new_forward(cch: &'a CCH, forward_cch_weights: Vec<(Weight, Weight)>, backward_cch_weights: Vec<(Weight, Weight)>) -> Self {
        let forward_cch_graph = UnweightedFirstOutGraph::new(cch.forward_first_out(), cch.forward_head());
        let backward_cch_graph = UnweightedFirstOutGraph::new(cch.backward_first_out(), cch.backward_head());
        let n = forward_cch_graph.num_nodes();

        Self {
            cch,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            forward_cch_graph,
            forward_cch_weights,
            backward_distances: TimestampedVector::new(n),
            backward_cch_graph,
            backward_cch_weights,
            num_pot_computations: 0,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }

    pub fn potential_bounds(&mut self, node: NodeId) -> Option<(Weight, Weight)> {
        let node = self.cch.node_order().rank(node);

        // upward search until a node with existing distance to target is found
        let mut cur_node = node;
        while self.potentials[cur_node as usize].value().is_none() {
            self.num_pot_computations += 1;
            self.stack.push(cur_node);
            if let Some(parent) = self.cch.elimination_tree()[cur_node as usize].value() {
                cur_node = parent;
            } else {
                break;
            }
        }

        // propagate the result back to the original start node
        while let Some(current_node) = self.stack.pop() {
            let (mut dist_lower, mut dist_upper) = self.backward_distances[current_node as usize];

            for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                let (edge_weight_lower, edge_weight_upper) = self.forward_cch_weights[edge as usize];
                let (next_potential_lower, next_potential_upper) = self.potentials[next_node as usize].value().unwrap();

                dist_lower = min(dist_lower, edge_weight_lower + next_potential_lower);
                dist_upper = min(dist_upper, edge_weight_upper + next_potential_upper);
            }

            self.potentials[current_node as usize] = InRangeOption::new(Some((dist_lower, dist_upper)));
        }

        let (dist_lower, dist_upper) = self.potentials[node as usize].value().unwrap();
        if dist_lower < INFINITY && dist_upper < INFINITY {
            Some((dist_lower, dist_upper))
        } else {
            None
        }
    }
}

impl<'a, CCH: CCHT> Potential for CCHLowerUpperPotential<'a, CCH> {
    fn init(&mut self, target: NodeId) {
        let target = self.cch.node_order().rank(target);
        self.potentials.reset();

        let mut bw_walk = CorridorEliminationTreeWalk::init(
            &self.backward_cch_graph,
            &self.backward_cch_weights,
            self.cch.elimination_tree(),
            &mut self.backward_distances,
            target,
        );

        while let Some(_) = bw_walk.next() {}
        self.num_pot_computations = 0;
    }

    fn potential(&mut self, node: NodeId) -> Option<u32> {
        self.potential_bounds(node).map(|(lower_bound, _)| lower_bound)
    }
}
