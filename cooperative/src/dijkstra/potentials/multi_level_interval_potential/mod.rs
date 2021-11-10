use crate::dijkstra::elimination_tree::multi_level_buckets::customized::CustomizedMultiLevels;
use crate::dijkstra::elimination_tree::multi_level_buckets::MultiLevelEliminationTreeWalk;
use crate::dijkstra::potentials::cch::custom_cch_pot::CCHLowerUpperPotential;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::cmp::min;

const LOWERBOUND_POT_THRESHOLD: Timestamp = 300_000; // = 5 minutes

pub struct CCHMultiLevelIntervalPotential<'a> {
    customized: &'a CustomizedMultiLevels<'a>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<Vec<Weight>>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Vec<Weight>>,
    backward_distances: TimestampedVector<Vec<Weight>>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Vec<Weight>>,
    forward_potential: CCHLowerUpperPotential<'a, CCH>,
    num_pot_computations: usize,
    current_metrics: Vec<usize>,
    current_intervals: Vec<Timestamp>,
    current_max_corridor: Weight,
    use_simple_pot: bool,
}

impl<'a> CCHMultiLevelIntervalPotential<'a> {
    pub fn new_forward(customized: &'a CustomizedMultiLevels<'a>) -> Self {
        let (forward_cch_graph, forward_cch_weights) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        // initialize lowerbound forward potential for interval query (also as fallback potential on tight corridors!)
        let pot_forward_weights = forward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();
        let pot_backward_weights = backward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();
        let forward_potential = CCHLowerUpperPotential::new_forward(customized.cch, pot_forward_weights, pot_backward_weights);

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            forward_potential,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            backward_distances: TimestampedVector::new(n),
            num_pot_computations: 0,
            current_metrics: Vec::new(),
            current_intervals: Vec::new(),
            current_max_corridor: MAX_BUCKETS,
            use_simple_pot: false,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }
}

impl<'a> TDPotential for CCHMultiLevelIntervalPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.current_metrics.clear();
        self.current_intervals.clear();
        self.forward_potential.init(source, target, timestamp);

        // 1st fallback: metric only contains lowerbound-weights
        if self.customized.bucket_tree.elements.len() == 1 {
            self.use_simple_pot = true;
            return;
        } else {
            self.use_simple_pot = false;
        }

        // 1. use interval query to determine corridor (for now, only the latest arrival is relevant)
        let corridor = self
            .forward_potential
            .potential_bounds(source)
            .map(|(arrival_lower, arrival_upper)| (timestamp + arrival_lower, timestamp + arrival_upper));

        if let Some((corridor_lower, corridor_upper)) = corridor {
            // 2nd fallback: use lowerbound pot if the corridor is too tight
            if corridor_upper - corridor_lower <= LOWERBOUND_POT_THRESHOLD {
                self.use_simple_pot = true;
                return;
            }

            // bound the upper corridor to the valid time range (lower is ok by query construction!)
            self.current_max_corridor = corridor_upper % MAX_BUCKETS;

            // 2. determine relevant metrics
            let tree = &self.customized.bucket_tree;

            // get the relevant start metric -> try to keep the intervals as small as possible -> every additionally required level decreases performance!
            let mut current_interval = Some(tree.find_interval(timestamp, self.current_max_corridor).0);

            while let Some(tree_idx) = current_interval {
                self.current_metrics.push(tree.elements[tree_idx].metric_id);
                self.current_intervals.push(tree.elements[tree_idx].interval_start);

                // find first suitable child
                current_interval = None;

                for child in tree.children_range(tree_idx) {
                    if tree.elements[child].interval_start <= self.current_max_corridor && tree.elements[child].interval_end > self.current_max_corridor {
                        current_interval = Some(child);
                        break;
                    }
                }

                /*current_interval = tree
                .children_range(tree_idx)
                .filter(|&child| {
                    tree.elements[child].interval_start <= self.current_max_corridor && tree.elements[child].interval_end > self.current_max_corridor
                })
                .next();*/
            }

            // 3. initialize custom elimination tree
            let target = self.customized.cch.node_order().rank(target);
            self.potentials.reset();

            let mut bw_walk = MultiLevelEliminationTreeWalk::init(
                &self.backward_cch_graph,
                &self.backward_cch_weights,
                &self.current_metrics,
                self.customized.cch.elimination_tree(),
                &mut self.backward_distances,
                target,
            );

            while let Some(_) = bw_walk.next() {}
            self.num_pot_computations = 0;
        } else {
            panic!("Failed to initialize corridor!");
        }
    }

    fn potential(&mut self, node: u32, timestamp: u32) -> Option<u32> {
        // check if lowerbound potential suffices -> metrics have not been initialized!
        if self.use_simple_pot {
            self.forward_potential.potential(node, timestamp).filter(|&pot| pot < INFINITY)
        } else {
            // regular case - the induced corridor was too large, so a fine-grained approach
            // improves the potential quality significantly
            let node = self.customized.cch.node_order.rank(node);
            let timestamp = timestamp % MAX_BUCKETS;
            let elimination_tree = self.customized.cch.elimination_tree();

            // 1. upward search until a node with existing distance to target is found
            // it suffices to check for the lowerbound distance which is always found at index 0
            let mut cur_node = node;
            while self.potentials[cur_node as usize][0] >= INFINITY {
                self.num_pot_computations += 1;
                self.stack.push(cur_node);
                if let Some(parent) = elimination_tree[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // 2. propagate the result back to the original start node
            while let Some(current_node) = self.stack.pop() {
                let distances = &mut self.backward_distances[current_node as usize];

                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                    let forward_cch_weights = &self.forward_cch_weights[edge as usize];
                    let next_potential = &self.potentials[next_node as usize];

                    /*distances
                    .iter_mut()
                    .zip(self.current_metrics.iter())
                    .zip(next_potential.iter())
                    .for_each(|((dist, &metric_id), &next_pot)| *dist = min(*dist, forward_cch_weights[metric_id] + next_pot));*/

                    for idx in 0..self.current_metrics.len() {
                        distances[idx] = min(distances[idx], forward_cch_weights[self.current_metrics[idx]] + next_potential[idx]);
                    }

                    /*self.current_metrics.iter().enumerate().for_each(|(idx, &metric_idx)| {
                        distances[idx] = min(distances[idx], forward_cch_weights[metric_idx] + next_potential[idx]);
                    });*/
                }

                self.potentials[current_node as usize] = distances.clone();
            }

            // 3. retrieve the value within the most-suited interval
            let pot_metric_idx = if timestamp > self.current_max_corridor {
                // special-case treatment for changes over midnight, simply use the lowerbound pot here
                0
            } else {
                // lookup for smallest bucket that contains both `timestamp` and the upper bound arrival
                let mut val = 0;

                for i in (0..self.current_intervals.len()).rev() {
                    if self.current_intervals[i] <= timestamp {
                        val = i;
                        break;
                    }
                }
                val
            };

            /*let potential_metric_idx = self
            .current_metrics
            .iter()
            .enumerate()
            .filter(|&(_, &m_idx)| m_idx == metric_idx)
            .next()
            .map(|(val, _)| val);*/

            Some(self.potentials[node as usize][pot_metric_idx]).filter(|&pot| pot < INFINITY)
        }
    }
}

/*
pub fn potential_bounds(&mut self, node: NodeId) -> Option<(Weight, Weight)> {
        let node = self.customized.cch.node_order().rank(node);

        // upward search until a node with existing distance to target is found
        let mut cur_node = node;
        while self.potentials[cur_node as usize].value().is_none() {
            self.num_pot_computations += 1;
            self.stack.push(cur_node);
            if let Some(parent) = self.customized.cch.elimination_tree()[cur_node as usize].value() {
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
 */
