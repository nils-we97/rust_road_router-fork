use crate::dijkstra::elimination_tree::multi_level_buckets::bucket_tree::{BucketTreeEntry, MultiLevelBucketTree};
use crate::dijkstra::elimination_tree::parallelization::SeparatorBasedParallelCustomization;
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

const LOWERBOUND_METRIC: usize = 0;
const UPPERBOUND_METRIC: usize = 1;

pub struct CustomizedMultiLevels<'a> {
    pub cch: &'a CCH,
    pub upward: Vec<Vec<Weight>>,
    pub downward: Vec<Vec<Weight>>,
    pub bucket_tree: MultiLevelBucketTree,
}

impl<'a> CustomizedMultiLevels<'a> {
    pub fn new(cch: &'a CCH, departures: &Vec<Vec<Timestamp>>, travel_times: &Vec<Vec<Weight>>, interval_lengths: &Vec<Timestamp>) -> Self {
        let m = cch.num_arcs();

        debug_assert!(!interval_lengths.is_empty(), "Intervals must not be empty!");
        debug_assert!(interval_lengths[0] < MAX_BUCKETS, "First interval length must be smaller than 24 hours!");
        debug_assert!(MAX_BUCKETS % interval_lengths[0] == 0, "Interval lengths must be divisible by each other!");

        interval_lengths
            .windows(2)
            .for_each(|a| debug_assert!(a[0] % a[1] == 0, "Interval lengths must be divisible by each other!"));

        // 1. extract the lower bounds for all intervals and store them in a consecutive array
        // build prefix sum over metric indices; first two entries are reserved for lower/upper bound!
        let mut level_boundaries = Vec::with_capacity(interval_lengths.len() + 1);
        level_boundaries.push(2);
        interval_lengths
            .iter()
            .for_each(|&interval_len| level_boundaries.push((MAX_BUCKETS / interval_len) + *level_boundaries.last().unwrap()));
        let metrics = extract_metrics(departures, travel_times, interval_lengths, &level_boundaries);

        // 2. create bucket tree structure for fast potential accesses
        let bucket_tree = build_bucket_tree(interval_lengths, &level_boundaries);

        // these will contain our customized shortcuts
        let mut upward_weights = vec![vec![INFINITY; metrics.len()]; m];
        let mut downward_weights = vec![vec![INFINITY; metrics.len()]; m];

        // 2. initialize upward and downward weights with correct lower/upper bound
        prepare_weights(cch, &mut upward_weights, &mut downward_weights, &metrics);

        // 3. run basic customization
        customize_basic(cch, &mut upward_weights, &mut downward_weights);

        println!("Sizes after basic customization: {} {}", upward_weights.len(), downward_weights.len());

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
fn extract_metrics(
    departures: &Vec<Vec<Timestamp>>,
    travel_times: &Vec<Vec<Weight>>,
    interval_lengths: &Vec<Timestamp>,
    level_boundaries: &Vec<u32>,
) -> Vec<Vec<Weight>> {
    // 1. initialize metric array and determine boundaries (prefix sum) for each level
    // first two entries are reserved for lower and upper bound potentials for the whole day

    // todo remove this block
    let num_entries = interval_lengths.iter().map(|&interval_len| MAX_BUCKETS / interval_len).sum::<u32>() + 2;
    debug_assert_eq!(num_entries, interval_lengths.last().unwrap().clone());

    let mut metrics = vec![vec![INFINITY; *interval_lengths.last().unwrap() as usize]; departures.len()];

    // 2. collect the interval minima from all edges
    (0..departures.len()).into_iter().for_each(|edge_id| {
        // 2a. collect upper and lower bound from travel times
        metrics[edge_id][LOWERBOUND_METRIC] = travel_times[edge_id].iter().min().unwrap().clone();
        metrics[edge_id][UPPERBOUND_METRIC] = travel_times[edge_id].iter().max().unwrap().clone();

        // 2b. collecting the other lowerbound weights also requires the departure timestamps
        debug_assert_eq!(departures[edge_id].len(), travel_times[edge_id].len());
        departures[edge_id][..departures[edge_id].len() - 1]
            .iter()
            .zip(travel_times[edge_id][..travel_times[edge_id].len() - 1].iter())
            .for_each(|(&departure, &travel_time)| {
                // retrieve the respective metric entry on all levels for the current departure,
                // update minimum value if necessary
                (0..interval_lengths.len()).into_iter().for_each(|level| {
                    let metric_idx = level_boundaries[level] + (departure / interval_lengths[level]);
                    metrics[edge_id][metric_idx as usize] = min(metrics[edge_id][metric_idx as usize], travel_time);
                });
            });

        // 2c. miminum values could also be at the start/end of the current interval
        // -> interpolate in order to consider otherwise missing entries!
        let plf = PiecewiseLinearFunction::new(&departures[edge_id], &travel_times[edge_id]);

        level_boundaries.windows(2).enumerate().for_each(|(level, bounds)| {
            (bounds[0]..bounds[1]).into_iter().for_each(|metric_idx| {
                let plf_at_begin = plf.eval(interval_lengths[level] * (metric_idx - bounds[0]));
                let plf_at_end = plf.eval(interval_lengths[level] * (metric_idx + 1 - bounds[1]));

                metrics[edge_id][metric_idx as usize] = min(metrics[edge_id][metric_idx as usize], min(plf_at_begin, plf_at_end));
            });
        });
    });

    metrics
}

fn build_bucket_tree(interval_lengths: &Vec<Timestamp>, level_boundaries: &Vec<u32>) -> MultiLevelBucketTree {
    let mut current_level_entries = Vec::new();
    let mut prev_level_entries = Vec::new();

    // iterate through all levels, start on the lowest
    level_boundaries.windows(2).enumerate().rev().for_each(|(idx, bounds)| {
        let num_entries_on_current_level = MAX_BUCKETS / interval_lengths[idx];
        let num_children = if idx == interval_lengths.len() - 1 {
            0
        } else {
            (interval_lengths[idx] / interval_lengths[idx + 1]) as usize
        };

        prev_level_entries = current_level_entries.clone();
        current_level_entries.clear();

        for i in 0..num_entries_on_current_level as usize {
            let mut children = Vec::new();
            if num_children > 0 {
                children.extend_from_slice(&prev_level_entries[(i * num_children)..((i + 1) * num_children)]);
            }
            current_level_entries.push(BucketTreeEntry::new(
                interval_lengths[idx] * (i as u32),
                interval_lengths[idx] * ((i + 1) as u32),
                (bounds[0] as usize) + i,
                children,
            ));
        }
    });

    debug_assert_eq!(current_level_entries.len(), (MAX_BUCKETS / interval_lengths[0]) as usize);

    // create root node, combine all recently added nodes
    let root = BucketTreeEntry::new(0, MAX_BUCKETS, LOWERBOUND_METRIC, current_level_entries);

    dbg!(&root);
    MultiLevelBucketTree::new(root)
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
