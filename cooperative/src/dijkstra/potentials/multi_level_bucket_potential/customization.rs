use crate::dijkstra::potentials::cch_parallelization_util::SeparatorBasedParallelCustomization;
use crate::dijkstra::potentials::multi_level_bucket_potential::bucket_tree::MultiLevelBucketTree;
use crate::graph::MAX_BUCKETS;
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, Reversed, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::report::{report_time, report_time_with_key};
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::min;
use std::ops::Range;

// One mapping of node id to weight for each thread during the scope of the customization.
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<Vec<Weight>>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<Vec<Weight>>>);

const UPPERBOUND_METRIC: usize = 1;

pub struct CustomizedMultiLevels<'a> {
    pub cch: &'a CCH,
    pub upward: Vec<Vec<Weight>>,
    pub downward: Vec<Vec<Weight>>,
    pub bucket_tree: MultiLevelBucketTree,
}

impl<'a> CustomizedMultiLevels<'a> {
    pub fn new(
        cch: &'a CCH,
        departures: &Vec<Vec<Timestamp>>,
        travel_times: &Vec<Vec<Weight>>,
        interval_lengths: &Vec<Timestamp>,
        reduction_threshold: u64,
    ) -> Self {
        let m = cch.num_arcs();

        debug_assert!(!interval_lengths.is_empty(), "Intervals must not be empty!");
        debug_assert!(interval_lengths[0] < MAX_BUCKETS, "First interval length must be smaller than 24 hours!");
        debug_assert!(MAX_BUCKETS % interval_lengths[0] == 0, "Interval lengths must be divisible by each other!");

        interval_lengths
            .windows(2)
            .for_each(|a| debug_assert!(a[0] % a[1] == 0, "Interval lengths must be divisible by each other!"));

        // 1. build bucket tree structure
        let mut bucket_tree = MultiLevelBucketTree::new(interval_lengths);

        // 2. extract the metrics
        let mut metrics = extract_metrics(departures, travel_times, &bucket_tree);

        // 3. reduce the number of metrics by applying some simple heuristics
        //let reduce_threshold = departures.len() as u64 * 20000;
        bucket_tree.reduce(reduction_threshold, &mut metrics);
        println!("Reduced similar metrics - remaining: {}", metrics[0].len());
        //dbg!(&bucket_tree);

        // these will contain our customized shortcuts
        let mut upward_weights = vec![vec![INFINITY; metrics[0].len()]; m];
        let mut downward_weights = vec![vec![INFINITY; metrics[0].len()]; m];

        // 4. initialize upward and downward weights with correct lower/upper bound
        prepare_weights(cch, &mut upward_weights, &mut downward_weights, &metrics);

        // 5. run basic customization
        customize_basic(cch, &mut upward_weights, &mut downward_weights);

        Self {
            cch,
            upward: upward_weights,
            downward: downward_weights,
            bucket_tree,
        }
    }

    pub fn forward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Vec<Weight>>) {
        (
            UnweightedFirstOutGraph::new(self.cch.forward_first_out(), self.cch.forward_head()),
            &self.upward,
        )
    }

    pub fn backward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Vec<Weight>>) {
        (
            UnweightedFirstOutGraph::new(self.cch.backward_first_out(), self.cch.backward_head()),
            &self.downward,
        )
    }
}

// subroutines
fn extract_metrics(departures: &Vec<Vec<Timestamp>>, travel_times: &Vec<Vec<Weight>>, tree: &MultiLevelBucketTree) -> Vec<Vec<Weight>> {
    let mut metrics = vec![vec![INFINITY; tree.elements.len() + 1]; departures.len()];

    // collect the metrics edge by edge; this layout is also needed by the customization step
    (0..departures.len()).into_iter().for_each(|edge_id| {
        // collect upper bound weights (lower bound will is done below!)
        metrics[edge_id][UPPERBOUND_METRIC] = travel_times[edge_id].iter().max().unwrap().clone();

        // for every other metric, we also need the departure timestamps
        departures[edge_id][..departures[edge_id].len() - 1]
            .iter()
            .zip(travel_times[edge_id][..travel_times[edge_id].len() - 1].iter())
            .for_each(|(&departure, &travel_time)| {
                // retrieve the respective metric entry on all levels for the current departure,
                // update minimum value if necessary

                // use a more general approach here, even if direct indexing would be possible
                // start with the root node and continuously descend to the correct children
                let mut current_node = Some(0);

                while let Some(node) = current_node {
                    let current_metric_id = tree.elements[node].metric_id;
                    metrics[edge_id][current_metric_id] = min(metrics[edge_id][current_metric_id], travel_time);

                    current_node = tree
                        .children_range(node)
                        .into_iter()
                        .filter(|&child| tree.elements[child].interval_start <= departure && tree.elements[child].interval_end > departure)
                        .next();
                }
            });

        // miminum values could also be at the start/end of the current interval
        // -> interpolate in order to consider otherwise missing entries!
        let plf = PiecewiseLinearFunction::new(&departures[edge_id], &travel_times[edge_id]);

        tree.elements.iter().for_each(|element| {
            let plf_at_begin = plf.eval(element.interval_start);
            let plf_at_end = plf.eval(element.interval_end);

            metrics[edge_id][element.metric_id] = min(metrics[edge_id][element.metric_id], min(plf_at_begin, plf_at_end));
        });
    });

    metrics
}

fn prepare_weights(cch: &CCH, upward_weights: &mut Vec<Vec<Weight>>, downward_weights: &mut Vec<Vec<Weight>>, metric: &Vec<Vec<Weight>>) {
    report_time("Apply weights", || {
        upward_weights
            .par_iter_mut()
            .zip(downward_weights.par_iter_mut())
            .zip(cch.cch_edge_to_orig_arc.par_iter())
            .for_each(|((upward, downward), (up_arcs, down_arcs))| {
                for metric_idx in 0..upward.len() {
                    for &EdgeIdT(up_arc) in up_arcs {
                        upward[metric_idx] = min(upward[metric_idx], metric[up_arc as usize][metric_idx]);
                    }
                    for &EdgeIdT(down_arc) in down_arcs {
                        downward[metric_idx] = min(downward[metric_idx], metric[down_arc as usize][metric_idx]);
                    }
                }
            });
    });
}

fn customize_basic(cch: &CCH, upward_weights: &mut Vec<Vec<Weight>>, downward_weights: &mut Vec<Vec<Weight>>) {
    let n = cch.num_nodes() as NodeId;
    let num_metrics = upward_weights[0].len();

    let customize = |nodes: Range<usize>, offset: usize, upward_weights: &mut [Vec<Weight>], downward_weights: &mut [Vec<Weight>]| {
        UPWARD_WORKSPACE.with(|node_outgoing_weights| {
            let mut node_outgoing_weights = node_outgoing_weights.borrow_mut();

            DOWNWARD_WORKSPACE.with(|node_incoming_weights| {
                let mut node_incoming_weights = node_incoming_weights.borrow_mut();

                for current_node in nodes {
                    let current_node = current_node as NodeId;
                    let mut edges = cch.neighbor_edge_indices_usize(current_node);
                    edges.start -= offset;
                    edges.end -= offset;
                    for ((node, down), up) in cch
                        .neighbor_iter(current_node)
                        .zip(&downward_weights[edges.clone()])
                        .zip(&upward_weights[edges.clone()])
                    {
                        node_incoming_weights[node as usize] = down.clone();
                        node_outgoing_weights[node as usize] = up.clone();
                    }

                    for (NodeIdT(low_node), Reversed(EdgeIdT(first_edge_id))) in cch.inverted.link_iter(current_node) {
                        let first_down_weight = &downward_weights[first_edge_id as usize - offset];
                        let first_up_weight = &upward_weights[first_edge_id as usize - offset];
                        let mut low_up_edges = cch.neighbor_edge_indices_usize(low_node);
                        low_up_edges.start -= offset;
                        low_up_edges.end -= offset;
                        for ((node, upward_weight), downward_weight) in cch
                            .neighbor_iter(low_node)
                            .rev()
                            .zip(upward_weights[low_up_edges.clone()].iter().rev())
                            .zip(downward_weights[low_up_edges].iter().rev())
                        {
                            if node <= current_node {
                                break;
                            }

                            let relax = unsafe { node_outgoing_weights.get_unchecked_mut(node as usize) };
                            for i in 0..relax.len() {
                                relax[i] = min(relax[i], upward_weight[i] + first_down_weight[i]);
                            }

                            let relax = unsafe { node_incoming_weights.get_unchecked_mut(node as usize) };
                            for i in 0..relax.len() {
                                relax[i] = min(relax[i], downward_weight[i] + first_up_weight[i]);
                            }
                        }
                    }

                    for (((node, down), up), _edge_id) in cch
                        .neighbor_iter(current_node)
                        .zip(&mut downward_weights[edges.clone()])
                        .zip(&mut upward_weights[edges.clone()])
                        .zip(edges)
                    {
                        *down = node_incoming_weights[node as usize].clone();
                        *up = node_outgoing_weights[node as usize].clone();
                    }
                }
            });
        });
    };

    // setup customization for parallelization
    let customization = SeparatorBasedParallelCustomization::new(cch, customize, customize);

    // execute customization
    report_time_with_key("CCH Customization", "basic_customization", || {
        customization.customize(upward_weights, downward_weights, |cb| {
            // create workspace vectors for the scope of the customization
            UPWARD_WORKSPACE.set(&RefCell::new(vec![vec![INFINITY; num_metrics]; n as usize]), || {
                DOWNWARD_WORKSPACE.set(&RefCell::new(vec![vec![INFINITY; num_metrics]; n as usize]), || cb());
            });
            // everything will be dropped here
        });
    });
}
