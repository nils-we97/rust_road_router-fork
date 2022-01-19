use crate::dijkstra::potentials::cch_lower_upper::elimination_tree_server::CorridorEliminationTreeServer;
use crate::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::datastr::timestamped_vector::TimestampedVector;
use rust_road_router::util::in_range_option::InRangeOption;
use std::cmp::min;

pub struct MultiMetricPotential<'a> {
    customized: CustomizedMultiMetrics<'a>,
    forward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    backward_cch_graph: UnweightedFirstOutGraph<&'a [EdgeId], &'a [NodeId]>,
    interval_query_server: CorridorEliminationTreeServer<'a, CCH, Vec<(Weight, Weight)>>,
    context: MultiLevelBucketPotentialContext,
}

struct MultiLevelBucketPotentialContext {
    stack: Vec<NodeId>,
    potentials: TimestampedVector<InRangeOption<Weight>>,
    backward_distances: TimestampedVector<Weight>,
    current_metric: usize,
    latest_arrival_dist: Option<Weight>,
    query_start: Timestamp,
    num_pot_computations: usize,
}

impl<'a> MultiMetricPotential<'a> {
    pub fn new(customized: CustomizedMultiMetrics<'a>) -> Self {
        let forward_cch_graph = UnweightedFirstOutGraph::new(&customized.cch.forward_first_out()[..], &customized.cch.forward_head()[..]);
        let backward_cch_graph = UnweightedFirstOutGraph::new(&customized.cch.backward_first_out()[..], &customized.cch.backward_head()[..]);

        let n = customized.cch.num_nodes();
        let m = customized.cch.num_arcs();

        let interval_query_server = Self::init_elimination_tree_server(
            customized.cch,
            &customized.upward[..m],
            &customized.upward[m..2 * m],
            &customized.downward[..m],
            &customized.downward[m..2 * m],
        );

        let context = MultiLevelBucketPotentialContext {
            stack: Vec::new(),
            potentials: TimestampedVector::new(n),
            backward_distances: TimestampedVector::new(n),
            current_metric: 0,
            latest_arrival_dist: None,
            query_start: 0,
            num_pot_computations: 0,
        };

        Self {
            customized,
            forward_cch_graph,
            backward_cch_graph,
            interval_query_server,
            context,
        }
    }

    pub fn num_pot_computations(&self) -> usize {
        self.context.num_pot_computations
    }

    fn init_elimination_tree_server(
        cch: &'a CCH,
        upward_lower: &[Weight],
        upward_upper: &[Weight],
        downward_lower: &[Weight],
        downward_upper: &[Weight],
    ) -> CorridorEliminationTreeServer<'a, CCH, Vec<(Weight, Weight)>> {
        let m = cch.num_arcs();

        let forward_graph = UnweightedFirstOutGraph::new(&cch.forward_first_out()[..], &cch.forward_head()[..]);
        let backward_graph = UnweightedFirstOutGraph::new(&cch.backward_first_out()[..], &cch.backward_head()[..]);

        let pot_forward_weights = (0..m)
            .into_iter()
            .map(|edge_id| (upward_lower[edge_id], upward_upper[edge_id]))
            .collect::<Vec<(Weight, Weight)>>();

        let pot_backward_weights = (0..m)
            .into_iter()
            .map(|edge_id| (downward_lower[edge_id], downward_upper[edge_id]))
            .collect::<Vec<(Weight, Weight)>>();

        CorridorEliminationTreeServer::new(cch, forward_graph, pot_forward_weights, backward_graph, pot_backward_weights)
    }
}

impl<'a> TDPotential for MultiMetricPotential<'a> {
    fn init(&mut self, source: u32, target: u32, timestamp: u32) {
        self.context.num_pot_computations = 0;

        // 1. use interval query to determine the latest arrival and the relevant backward upward search space
        self.context.latest_arrival_dist = self.interval_query_server.query(source, target).map(|(_, upper)| upper);
        self.context.query_start = timestamp;

        if let Some(upper_arrival_dist) = self.context.latest_arrival_dist {
            // 2. determine relevant metric: find shortest interval that spans the start and latest arrival
            // tie-breaking: smaller metric id wins (i.e. firstly iterated over)
            let latest_arrival_ts = timestamp + upper_arrival_dist;
            if latest_arrival_ts >= MAX_BUCKETS {
                // special case treatment for queries whose arrival time possibly crosses midnight
                self.context.current_metric = 0; // use lowerbound
            } else {
                let mut tightest_interval_len = MAX_BUCKETS + 1;

                self.customized.metric_entries.iter().for_each(|entry| {
                    if entry.start <= timestamp && entry.end >= latest_arrival_ts && (entry.end - entry.start < tightest_interval_len) {
                        tightest_interval_len = entry.end - entry.start;
                        self.context.current_metric = entry.metric_id;
                    }
                });
            }

            // 3. intialize elimination tree, restrict to backward upward search space from interval query!
            let target = self.customized.cch.node_order().rank(target);
            let query_backward_distances = self.interval_query_server.backward_distances();
            self.context.potentials.reset();
            self.context.backward_distances.reset();
            self.context.backward_distances[target as usize] = 0;

            let mut current_node = Some(target);
            while let Some(node) = current_node {
                current_node = self.customized.cch.elimination_tree()[node as usize].value();

                // additional pruning: only relax edges if the backward distance label is set for this node!
                if query_backward_distances[node as usize].0 < INFINITY {
                    // For each node we can reach, see if we can find a way with
                    // a lower distance going through this node
                    for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.backward_cch_graph, node) {
                        let next_node = next_node as usize;

                        let weight = self.context.backward_distances[node as usize]
                            + *unsafe {
                                self.customized
                                    .downward
                                    .get_unchecked(self.context.current_metric * self.backward_cch_graph.num_arcs() + edge as usize)
                            };

                        self.context.backward_distances[next_node] = min(self.context.backward_distances[next_node], weight);
                    }
                }
            }
        }
    }

    fn potential(&mut self, node: NodeId, _timestamp: Timestamp) -> Option<Weight> {
        // if the target isn't reachable from the source, we can abort here
        if let Some(latest_arrival_dist) = self.context.latest_arrival_dist {
            let node = self.customized.cch.node_order.rank(node);
            let elimination_tree = self.customized.cch.elimination_tree();

            // 1. upward search until a node with existing distance to target is found
            let mut cur_node = node;
            while self.context.potentials[cur_node as usize].value().is_none() {
                self.context.num_pot_computations += 1;
                self.context.stack.push(cur_node);
                if let Some(parent) = elimination_tree[cur_node as usize].value() {
                    cur_node = parent;
                } else {
                    break;
                }
            }

            // 2. propagate the result back to the original start node
            while let Some(current_node) = self.context.stack.pop() {
                for (NodeIdT(next_node), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.forward_cch_graph, current_node) {
                    let weight = self.context.backward_distances[next_node as usize]
                        + *unsafe {
                            self.customized
                                .upward
                                .get_unchecked(self.context.current_metric * self.forward_cch_graph.num_arcs() + edge as usize)
                        };

                    self.context.backward_distances[current_node as usize] = min(self.context.backward_distances[current_node as usize], weight);
                }

                self.context.potentials[current_node as usize] = InRangeOption::some(self.context.backward_distances[current_node as usize]);
            }

            self.context.potentials[node as usize].value().filter(|&pot| pot <= latest_arrival_dist)
        } else {
            None
        }
    }

    fn verify_result(&self, distance: Weight) -> bool {
        distance == INFINITY || distance <= self.context.latest_arrival_dist.unwrap()
    }

    fn refresh_bounds(&mut self, graph: &CapacityGraph) {
        let m = self.customized.cch.num_arcs();
        let (upper_forward, upper_backward) = self.customized.coop_fix_upper_bound(graph);

        debug_assert_eq!(m, upper_forward.len());
        debug_assert_eq!(m, upper_backward.len());

        self.interval_query_server = Self::init_elimination_tree_server(
            self.customized.cch,
            &self.customized.upward[..m],
            &upper_forward,
            &self.customized.downward[..m],
            &upper_backward,
        );
    }
}
