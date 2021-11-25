use crate::dijkstra::elimination_tree::approximated_periodic_ttf::customized_catchup::customize_ptv_graph;
use crate::dijkstra::elimination_tree::parallelization::SeparatorBasedParallelCustomization;
use crate::dijkstra::potentials::{convert_timestamp_f64_to_u32, convert_timestamp_u32_to_f64};
use crate::graph::MAX_BUCKETS;
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::{CCH, CCHT};
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, PeriodicPiecewiseLinearFunction, TDGraph, TTFPoint, Timestamp, PLF};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, Reversed, UnweightedFirstOutGraph, INFINITY};
use rust_road_router::report::{report_time, report_time_with_key};
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::{max, min};
use std::ops::Range;

// One mapping of node id to weight for each thread during the scope of the customization.
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<Vec<TTFPoint>>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<Vec<TTFPoint>>>);

pub struct CustomizedApproximatedPeriodicTTF<'a> {
    pub cch: &'a CCH,
    pub upward_intervals: Vec<Vec<u32>>,
    pub downward_intervals: Vec<Vec<u32>>,
    pub upward_bounds: Vec<(u32, u32)>,
    pub downward_bounds: Vec<(u32, u32)>,
    pub num_intervals: u32,
}

impl<'a> CustomizedApproximatedPeriodicTTF<'a> {
    pub fn new(cch: &'a CCH, departures: &Vec<Vec<u32>>, travel_times: &Vec<Vec<u32>>, approximation_threshold: usize, num_intervals: u32) -> Self {
        debug_assert!(MAX_BUCKETS % num_intervals == 0);
        let m = cch.num_arcs();

        // 1. convert to floating point values
        let edge_profiles = departures
            .iter()
            .zip(travel_times.iter())
            .map(|(node_departure, node_travel_time)| {
                node_departure
                    .iter()
                    .zip(node_travel_time.iter())
                    .map(|(&ts, &val)| TTFPoint {
                        at: Timestamp::new(convert_timestamp_u32_to_f64(ts)),
                        val: FlWeight::new(convert_timestamp_u32_to_f64(val)),
                    })
                    .collect::<Vec<TTFPoint>>()
            })
            .collect::<Vec<Vec<TTFPoint>>>();

        // 2. initialize containers for customized shortcuts
        let mut upward_weights = vec![empty_ttf(); m];
        let mut downward_weights = vec![empty_ttf(); m];

        // 3. initialize upward and downward weights
        prepare_weights(cch, &mut upward_weights, &mut downward_weights, &edge_profiles, approximation_threshold);

        // 4. run basic customization
        customize_basic(cch, &mut upward_weights, &mut downward_weights, approximation_threshold);

        // 5. adjust weights back to integer values
        let upward_intervals = extract_interval_minima(&upward_weights, num_intervals);
        let downward_intervals = extract_interval_minima(&downward_weights, num_intervals);

        let upward_bounds = extract_bounds(&upward_weights);
        let downward_bounds = extract_bounds(&downward_weights);

        Self {
            cch,
            upward_intervals,
            downward_intervals,
            upward_bounds,
            downward_bounds,
            num_intervals,
        }
    }

    pub fn new_from_ptv(cch: &'a CCH, graph: &'a TDGraph, num_intervals: u32) -> Self {
        debug_assert!(MAX_BUCKETS % num_intervals == 0);

        let (upward_weights, downward_weights) = customize_ptv_graph(cch, graph);

        let upward_intervals = extract_interval_minima(&upward_weights, num_intervals);
        let downward_intervals = extract_interval_minima(&downward_weights, num_intervals);

        let upward_bounds = extract_bounds(&upward_weights);
        let downward_bounds = extract_bounds(&downward_weights);

        Self {
            cch,
            upward_intervals,
            downward_intervals,
            upward_bounds,
            downward_bounds,
            num_intervals,
        }
    }

    pub fn forward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Vec<u32>>, &Vec<(u32, u32)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.forward_first_out(), self.cch.forward_head()),
            &self.upward_intervals,
            &self.upward_bounds,
        )
    }

    pub fn backward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<Vec<u32>>, &Vec<(u32, u32)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.backward_first_out(), self.cch.backward_head()),
            &self.downward_intervals,
            &self.downward_bounds,
        )
    }
}

fn prepare_weights(
    cch: &CCH,
    upward_weights: &mut Vec<Vec<TTFPoint>>,
    downward_weights: &mut Vec<Vec<TTFPoint>>,
    metric: &Vec<Vec<TTFPoint>>,
    approximation_threshold: usize,
) {
    report_time("Apply weights", || {
        upward_weights
            .par_iter_mut()
            .zip(downward_weights.par_iter_mut())
            .zip(cch.cch_edge_to_orig_arc.par_iter())
            .for_each(|((upward, downward), (up_arcs, down_arcs))| {
                for &EdgeIdT(up_arc) in up_arcs {
                    let current_plf = PeriodicPiecewiseLinearFunction::new(upward);
                    let other_plf = PeriodicPiecewiseLinearFunction::new(&metric[up_arc as usize]);

                    let (result, _) = current_plf.merge(&other_plf, &mut Vec::new());

                    *upward = if result.len() > approximation_threshold {
                        PeriodicPiecewiseLinearFunction::new(&result)
                            .lower_bound_ttf(&mut Vec::new(), &mut Vec::new())
                            .to_vec()
                    } else {
                        result.to_vec()
                    };
                }
                for &EdgeIdT(down_arc) in down_arcs {
                    let current_plf = PeriodicPiecewiseLinearFunction::new(downward);
                    let other_plf = PeriodicPiecewiseLinearFunction::new(&metric[down_arc as usize]);

                    let (result, _) = current_plf.merge(&other_plf, &mut Vec::new());

                    *downward = if result.len() > approximation_threshold {
                        PeriodicPiecewiseLinearFunction::new(&result)
                            .lower_bound_ttf(&mut Vec::new(), &mut Vec::new())
                            .to_vec()
                    } else {
                        result.to_vec()
                    };
                }
            });
    });
}

fn customize_basic(cch: &CCH, upward_weights: &mut Vec<Vec<TTFPoint>>, downward_weights: &mut Vec<Vec<TTFPoint>>, approximation_threshold: usize) {
    let n = cch.num_nodes() as NodeId;

    let customize = |nodes: Range<usize>, offset: usize, upward_weights: &mut [Vec<TTFPoint>], downward_weights: &mut [Vec<TTFPoint>]| {
        //let mut merge_buffer = Vec::new();

        UPWARD_WORKSPACE.with(|node_outgoing_weights| {
            let mut node_outgoing_weights = node_outgoing_weights.borrow_mut();

            DOWNWARD_WORKSPACE.with(|node_incoming_weights| {
                let mut node_incoming_weights = node_incoming_weights.borrow_mut();

                let mut merge_buffer_1 = Vec::new();
                let mut merge_buffer_2 = Vec::new();

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
                            // link & merge
                            // todo which value to link upon which?
                            // node = y, low_node = z, current_node = x
                            // -> edge(x, z).link(edge(z, y)) = edge(current_node, low_node).link(edge(low_node, node))
                            // => edge(x, z) = first_up/first_down, edge(z, y) = upward_weight/downward_weight

                            // relax lower triangle: check if path (current_node -> low_node -> node)
                            // is faster than current weight along (current_node -> node)
                            let linked = PeriodicPiecewiseLinearFunction::new(first_down_weight).link(&PeriodicPiecewiseLinearFunction::new(upward_weight));
                            let (mut merge_result, _) =
                                PeriodicPiecewiseLinearFunction::new(relax).merge(&PeriodicPiecewiseLinearFunction::new(&linked), &mut Vec::new());

                            // approximate as long as there are too many breakpoints
                            let mut count = 0;
                            while merge_result.len() > approximation_threshold && count < 5 {
                                count += 1;
                                merge_result = PeriodicPiecewiseLinearFunction::new(&merge_result).lower_bound_ttf(&mut merge_buffer_1, &mut merge_buffer_2);
                            }
                            *relax = merge_result.to_vec();

                            // downward weights: other way around, as we're switching the edge direction! first down to the lower node, then upward to the current node
                            let relax = unsafe { node_incoming_weights.get_unchecked_mut(node as usize) };
                            let linked = PeriodicPiecewiseLinearFunction::new(downward_weight).link(&PeriodicPiecewiseLinearFunction::new(first_up_weight));

                            let (mut merge_result, _) =
                                PeriodicPiecewiseLinearFunction::new(relax).merge(&PeriodicPiecewiseLinearFunction::new(&linked), &mut Vec::new());

                            count = 0;
                            while merge_result.len() > approximation_threshold && count < 5 {
                                count += 1;
                                merge_result = PeriodicPiecewiseLinearFunction::new(&merge_result).lower_bound_ttf(&mut merge_buffer_1, &mut merge_buffer_2);
                            }
                            *relax = merge_result.to_vec();
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
            UPWARD_WORKSPACE.set(&RefCell::new(vec![empty_ttf(); n as usize]), || {
                DOWNWARD_WORKSPACE.set(&RefCell::new(vec![empty_ttf(); n as usize]), || cb());
            });
            // everything will be dropped here
        });
    });
}

fn extract_interval_minima(weights: &Vec<Vec<TTFPoint>>, num_intervals: u32) -> Vec<Vec<u32>> {
    let interval_length = MAX_BUCKETS / num_intervals;

    weights
        .iter()
        .map(|ttf| {
            // collect minima within the current interval
            let mut interval_min = vec![INFINITY; num_intervals as usize];
            let mut bucket_idx = 0;

            // deal with constant functions
            let ttf = if ttf.last().unwrap().at.fuzzy_lt(Timestamp::new(86400.0)) {
                debug_assert_eq!(ttf.len(), 1, "Expected constant function, got: {:#?}", &ttf);
                let val = ttf.first().unwrap().val;
                vec![
                    TTFPoint { at: Timestamp::ZERO, val },
                    TTFPoint {
                        at: Timestamp::new(86400.0),
                        val,
                    },
                ]
            } else {
                ttf.clone()
            };

            debug_assert!(
                ttf.last().unwrap().at.fuzzy_eq(Timestamp::new(86400.0)),
                "last value: {}",
                ttf.last().unwrap().at.0
            );

            ttf[..ttf.len() - 1].iter().for_each(|point| {
                let ts = convert_timestamp_f64_to_u32(point.at.0);

                while ts >= (bucket_idx + 1) * interval_length {
                    bucket_idx += 1;
                    debug_assert!(bucket_idx < num_intervals, "sentinel must not be exceeded!, timestamp: {}", ts);
                }

                interval_min[bucket_idx as usize] = min(interval_min[bucket_idx as usize], convert_timestamp_f64_to_u32(point.val.0));
            });

            // also collect values at interval borders
            let plf = PeriodicPiecewiseLinearFunction::new(&ttf);
            interval_min.iter_mut().enumerate().for_each(|(idx, val)| {
                let ts = convert_timestamp_u32_to_f64((idx as u32) * interval_length);
                let ts_next = convert_timestamp_u32_to_f64((idx as u32 + 1) * interval_length);

                let bucket_start = plf.evaluate(Timestamp::new(ts));
                let bucket_end = plf.evaluate(Timestamp::new(ts_next));

                *val = min(
                    *val,
                    min(convert_timestamp_f64_to_u32(bucket_start.0), convert_timestamp_f64_to_u32(bucket_end.0)),
                );
            });

            interval_min
        })
        .collect::<Vec<Vec<u32>>>()
}

fn extract_bounds(weights: &Vec<Vec<TTFPoint>>) -> Vec<(u32, u32)> {
    weights
        .iter()
        .map(|ttf| {
            let mut min_val = INFINITY;
            let mut max_val = 0;

            ttf.iter().for_each(|point| {
                min_val = min(min_val, convert_timestamp_f64_to_u32(point.val.0));
                max_val = max(max_val, convert_timestamp_f64_to_u32(point.val.0));
            });

            (min(min_val, INFINITY), min(max_val, INFINITY))
        })
        .collect::<Vec<(u32, u32)>>()
}

fn empty_ttf() -> Vec<TTFPoint> {
    vec![
        TTFPoint {
            at: Timestamp::ZERO,
            val: FlWeight::INFINITY,
        },
        TTFPoint {
            at: Timestamp(86400.0),
            val: FlWeight::INFINITY,
        },
    ]
}

#[test]
fn test_interval_minima() {
    let weights = vec![vec![
        TTFPoint {
            at: Timestamp(0.0),
            val: FlWeight(100.0),
        },
        TTFPoint {
            at: Timestamp(43200.0 - 100.0),
            val: FlWeight(100.0),
        },
        TTFPoint {
            at: Timestamp(43200.0),
            val: FlWeight(200.0),
        },
        TTFPoint {
            at: Timestamp(61000.0 - 50.0),
            val: FlWeight(200.0),
        },
        TTFPoint {
            at: Timestamp(61000.0),
            val: FlWeight(250.0),
        },
        TTFPoint {
            at: Timestamp(86400.0 - 150.0),
            val: FlWeight(250.0),
        },
        TTFPoint {
            at: Timestamp(86400.0),
            val: FlWeight(100.0),
        },
    ]];
    let result = extract_interval_minima(&weights, 4);
    dbg!(&result);
}
