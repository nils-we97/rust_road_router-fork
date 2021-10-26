use crate::dijkstra::elimination_tree::corridor_intervals::server::CorridorEliminationTreeServer;
use crate::dijkstra::elimination_tree::multi_level_buckets::customized::CustomizedMultiLevels;
use crate::dijkstra::elimination_tree::multi_level_buckets::MultiLevelEliminationTreeWalk;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use std::cmp::min;

pub struct CCHMultiLevelIntervalPotential<'a> {
    customized: &'a CustomizedMultiLevels<'a>,
    stack: Vec<NodeId>,
    potentials: TimestampedVector<Vec<Weight>>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    forward_cch_weights: &'a Vec<Vec<Weight>>,
    backward_distances: TimestampedVector<Vec<Weight>>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_weights: &'a Vec<Vec<Weight>>,
    interval_query_server: CorridorEliminationTreeServer<'a, CCH>,
    num_pot_computations: usize,
    current_metrics: Vec<usize>,
    current_intervals: Vec<Timestamp>,
    current_max_corridor: Weight,
}

impl<'a> CCHMultiLevelIntervalPotential<'a> {
    pub fn new_forward(customized: &'a CustomizedMultiLevels<'a>, num_levels: usize) -> Self {
        let (forward_cch_graph, forward_cch_weights) = customized.forward_graph();
        let (backward_cch_graph, backward_cch_weights) = customized.backward_graph();
        let n = forward_cch_graph.num_nodes();

        // initialize separate elimination tree server for interval queries
        let interval_query_forward_weights = forward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();
        let interval_query_backward_weights = backward_cch_weights
            .iter()
            .map(|weights| (weights[0], weights[1]))
            .collect::<Vec<(Weight, Weight)>>();
        let interval_query_server = CorridorEliminationTreeServer::new(
            customized.cch,
            forward_cch_graph.clone(),
            interval_query_forward_weights,
            backward_cch_graph.clone(),
            interval_query_backward_weights,
        );

        Self {
            customized,
            forward_cch_graph,
            forward_cch_weights,
            backward_cch_graph,
            backward_cch_weights,
            interval_query_server,
            stack: Vec::new(),
            potentials: TimestampedVector::new(n, vec![INFINITY; num_levels + 1]), // lowerbound + each level
            backward_distances: TimestampedVector::new(n, vec![INFINITY; num_levels + 1]),
            num_pot_computations: 0,
            current_metrics: Vec::new(),
            current_intervals: Vec::new(),
            current_max_corridor: MAX_BUCKETS,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.num_pot_computations
    }
}

impl<'a> TDPotential for CCHMultiLevelIntervalPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        // 1. use interval query to determine corridor
        let corridor = self
            .interval_query_server
            .query(source, target)
            .map(|(_, arrival_upper)| (timestamp, arrival_upper));

        if let Some((corridor_lower, corridor_upper)) = corridor {
            // bound the upper corridor to the valid time range (lower is ok by query construction!)
            self.current_max_corridor = corridor_upper % MAX_BUCKETS;

            // 2. determine relevant metrics
            // first, find the largest necessary interval to span the whole corridor => the smaller, the better!
            let root = &self.customized.bucket_tree.root;
            let mut current_interval = Some(root.find_interval(corridor_lower, self.current_max_corridor).unwrap_or(root));
            self.current_metrics.clear();
            self.current_intervals.clear();

            while let Some(interval) = current_interval {
                self.current_metrics.push(interval.metric_index);
                self.current_intervals.push(interval.interval_end - interval.interval_start);

                // find suitable child that contains the upper corridor
                current_interval = interval
                    .children
                    .iter()
                    .filter(|child| child.interval_start <= self.current_max_corridor && child.interval_end > self.current_max_corridor)
                    .next();
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
        let node = self.customized.cch.node_order.rank(node);
        let elimination_tree = self.customized.cch.elimination_tree();

        // 1. upward search until a node with existing distance to target is found
        let mut cur_node = node;
        while self.potentials[cur_node as usize].is_empty() {
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

            //let (mut dist_lower, mut dist_upper) = self.backward_distances[current_node as usize];

            for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                let forward_cch_weights = &self.forward_cch_weights[edge as usize];
                let next_potential = &self.potentials[next_node as usize];

                //let (edge_weight_lower, edge_weight_upper) = self.forward_cch_weights[edge as usize];
                //let (next_potential_lower, next_potential_upper) = self.potentials[next_node as usize].value().unwrap();

                self.current_metrics.iter().enumerate().for_each(|(idx, &metric_idx)| {
                    distances[idx] = min(distances[idx], forward_cch_weights[metric_idx] + next_potential[idx]);
                });
            }

            self.potentials[current_node as usize] = distances.clone();
        }

        // 3. retrieve the value within the most-suited interval
        let metric_idx = self
            .customized
            .bucket_tree
            .root
            .find_interval(timestamp, self.current_max_corridor)
            .map(|entry| entry.metric_index)
            .unwrap_or(0);

        let potential_metric_idx = self
            .current_metrics
            .iter()
            .enumerate()
            .filter(|&(_, &m_idx)| m_idx == metric_idx)
            .next()
            .map(|(val, _)| val);

        debug_assert!(
            potential_metric_idx.is_some(),
            "Existing indices: {:?}, expected {}",
            &self.current_metrics,
            metric_idx
        );

        let pot = self.potentials[node as usize][potential_metric_idx.unwrap()];
        if pot < INFINITY {
            Some(pot)
        } else {
            None
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
