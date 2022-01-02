use crate::dijkstra::potentials::cch_parallelization_util::SeparatorBasedParallelCustomization;
use crate::dijkstra::potentials::multi_metric_potential::metric_reduction::{reduce_metrics, MetricEntry};
use crate::graph::MAX_BUCKETS;
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, Reversed, UnweightedFirstOutGraph, Weight, INFINITY};
use rust_road_router::report::{measure, report_time, report_time_with_key};
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::min;
use std::ops::Range;

// One mapping of node id to weight for each thread during the scope of the customization.
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<Vec<Weight>>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<Vec<Weight>>>);

const LOWERBOUND_METRIC: usize = 0;
const UPPERBOUND_METRIC: usize = 1;

pub struct CustomizedMultiMetrics<'a> {
    pub cch: &'a CCH,
    pub upward: Vec<Weight>,
    pub downward: Vec<Weight>,
    pub metric_entries: Vec<MetricEntry>,
    pub num_metrics: usize,
}

impl<'a> CustomizedMultiMetrics<'a> {
    pub fn new(
        cch: &'a CCH,
        departures: &Vec<Vec<Timestamp>>,
        travel_times: &Vec<Vec<Weight>>,
        intervals: &Vec<(Timestamp, Timestamp)>,
        num_max_metrics: usize,
    ) -> Self {
        let m = cch.num_arcs();
        debug_assert!(!intervals.is_empty(), "Intervals must not be empty!");
        assert!(num_max_metrics >= 1, "At least one metric (lowerbound) must be kept!");

        // 1. build metric entry structure
        let mut metric_entries = build_metric_entries(intervals);

        // 2. extract the metrics
        let (mut metrics, time) = measure(|| extract_metrics(departures, travel_times, &metric_entries));
        println!("Extracting all metrics took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        // 3. reduce the number of metrics by merging similar intervals
        let (num_metrics, time) = measure(|| reduce_metrics(&mut metrics, &mut metric_entries, num_max_metrics));
        println!(
            "Reducing to {} metrics took {} ms",
            num_metrics,
            time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
        );

        // these will contain our customized shortcuts
        let mut upward_weights = vec![vec![INFINITY; num_metrics]; m];
        let mut downward_weights = vec![vec![INFINITY; num_metrics]; m];

        // 4. initialize upward and downward weights with correct lower/upper bound
        prepare_weights(cch, &mut upward_weights, &mut downward_weights, &metrics);

        // 5. run basic customization
        customize_basic(cch, &mut upward_weights, &mut downward_weights);

        // 6. reorder weights
        let upward_weights = reorder_weights(&upward_weights, num_metrics);
        let downward_weights = reorder_weights(&downward_weights, num_metrics);

        Self {
            cch,
            upward: upward_weights,
            downward: downward_weights,
            metric_entries,
            num_metrics,
        }
    }

    pub fn forward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Weight>) {
        (
            UnweightedFirstOutGraph::new(self.cch.forward_first_out(), self.cch.forward_head()),
            &self.upward,
        )
    }

    pub fn backward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Weight>) {
        (
            UnweightedFirstOutGraph::new(self.cch.backward_first_out(), self.cch.backward_head()),
            &self.downward,
        )
    }
}

// subroutines
fn build_metric_entries(intervals: &Vec<(Timestamp, Timestamp)>) -> Vec<MetricEntry> {
    let mut ret = vec![MetricEntry::new(0, MAX_BUCKETS, LOWERBOUND_METRIC)];

    intervals.iter().enumerate().for_each(|(idx, &(start, end))| {
        ret.push(MetricEntry::new(start, end, idx + 2));
    });

    ret
}

/// reorder weights, flatten the 2-dimensional vector into a single dimension
/// data by metric and edge_id is found at index `metric * num_edges + edge_id`
fn reorder_weights(weights: &Vec<Vec<Weight>>, num_metrics: usize) -> Vec<Weight> {
    let mut ret = vec![0; weights.len() * num_metrics];

    weights.iter().enumerate().for_each(|(edge_id, edge_weights)| {
        edge_weights.iter().enumerate().for_each(|(metric_id, &val)| {
            ret[metric_id * weights.len() + edge_id] = val;
        });
    });

    ret
}

fn extract_metrics(departures: &Vec<Vec<Timestamp>>, travel_times: &Vec<Vec<Weight>>, entries: &Vec<MetricEntry>) -> Vec<Vec<Weight>> {
    let mut metrics = vec![vec![INFINITY; entries.len() + 2]; departures.len()];

    // collect the metrics edge by edge; this layout is also needed by the customization step
    metrics.par_iter_mut().enumerate().for_each(|(edge_id, edge_metrics): (usize, &mut Vec<u32>)| {
        // collect upper bound weights (lowerbound is part of the entry structure and done below!)
        edge_metrics[UPPERBOUND_METRIC] = *travel_times[edge_id].iter().max().unwrap();

        // for the other metrics, we also need the departure timestamps
        departures[edge_id]
            .iter()
            .zip(travel_times[edge_id].iter())
            .for_each(|(&departure, &travel_time)| {
                // update the minimum value for all relevant intervals, i.e. those who include the current departure timestamp
                entries.iter().for_each(|entry| {
                    if entry.start <= departure && entry.end >= departure {
                        edge_metrics[entry.metric_id] = min(edge_metrics[entry.metric_id], travel_time);
                    }
                });
            });

        // miminum values could also be at the start/end of the current interval
        // -> interpolate in order to consider otherwise missing entries!
        let plf = PiecewiseLinearFunction::new(&departures[edge_id], &travel_times[edge_id]);

        entries.iter().for_each(|entry| {
            let plf_at_begin = plf.eval(entry.start);
            let plf_at_end = plf.eval(entry.end);

            edge_metrics[entry.metric_id] = min(edge_metrics[entry.metric_id], min(plf_at_begin, plf_at_end));
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