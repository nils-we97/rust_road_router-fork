use crate::dijkstra::potentials::corridor_lowerbound_potential::customization_catchup::customize_ptv_graph;
use crate::graph::MAX_BUCKETS;
use rust_road_router::algo::customizable_contraction_hierarchy::{DirectedCCH, CCH, CCHT};
use rust_road_router::datastr::graph::floating_time_dependent::{TDGraph, TTFPoint};
use rust_road_router::datastr::graph::{
    BuildReversed, EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, ReversedGraphWithEdgeIds, UnweightedFirstOutGraph, INFINITY,
};
use rust_road_router::report::measure;
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;

// One mapping of node id to weight for each thread during the scope of the customization.
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<Vec<TTFPoint>>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<Vec<TTFPoint>>>);

pub struct CustomizedApproximatedPeriodicTTF<CCH> {
    pub cch: CCH,
    pub upward_intervals: Vec<u32>,
    pub downward_intervals: Vec<u32>,
    pub upward_bounds: Vec<(u32, u32)>,
    pub downward_bounds: Vec<(u32, u32)>,
    pub num_intervals: u32,
}

/*impl<'a> CustomizedApproximatedPeriodicTTF<&'a CCH> {
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
}*/

impl CustomizedApproximatedPeriodicTTF<DirectedCCH> {
    pub fn new_from_ptv(cch: &CCH, graph: &TDGraph, num_intervals: u32) -> Self {
        debug_assert!(MAX_BUCKETS % num_intervals == 0);

        let ((mut upward_weights, mut downward_weights), time) = measure(|| customize_ptv_graph(cch, graph, num_intervals));
        println!("Interval Minima Customization took {} ms", time.as_secs_f64() * 1000.0);

        // extract relevant data
        let mut num_removed_edges = 0;
        let (mut upward_intervals, upward_bounds): (Vec<Vec<u32>>, Vec<(u32, u32)>) = upward_weights
            .iter_mut()
            .map(|wrapper| {
                if wrapper.shortcut.required && wrapper.bounds.0 <= wrapper.bounds.1 {
                    let ret = (wrapper.interval_minima.clone(), wrapper.bounds);
                    wrapper.interval_minima = vec![];
                    ret
                } else {
                    num_removed_edges += 1;
                    (vec![], (INFINITY, INFINITY))
                }
            })
            .unzip();

        println!(
            "Reduction in upward direction: Removed {} of {} edges.",
            num_removed_edges,
            upward_weights.len()
        );
        num_removed_edges = 0;

        let (mut downward_intervals, downward_bounds): (Vec<Vec<u32>>, Vec<(u32, u32)>) = downward_weights
            .iter_mut()
            .map(|wrapper| {
                if wrapper.shortcut.required && wrapper.bounds.0 <= wrapper.bounds.1 {
                    let ret = (wrapper.interval_minima.clone(), wrapper.bounds);
                    wrapper.interval_minima = vec![];
                    ret
                } else {
                    num_removed_edges += 1;
                    (vec![], (INFINITY, INFINITY))
                }
            })
            .unzip();
        println!(
            "Reduction in downward direction: Removed {} of {} edges.",
            num_removed_edges,
            downward_weights.len()
        );

        // build directed cch, remove unnecessary shortcuts
        // also directly flatten the interval structure
        let ((cch, upward_intervals, downward_intervals, upward_bounds, downward_bounds), time) = measure(|| {
            build_customized_graph(
                cch,
                &mut upward_intervals,
                &upward_bounds,
                &mut downward_intervals,
                &downward_bounds,
                num_intervals,
            )
        });
        println!("Re-Building new CCH graph took {} ms", time.as_secs_f64() * 1000.0);

        Self {
            cch,
            upward_intervals,
            downward_intervals,
            upward_bounds,
            downward_bounds,
            num_intervals,
        }
    }
}

impl<CCH: CCHT> CustomizedApproximatedPeriodicTTF<CCH> {
    pub fn forward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<u32>, &Vec<(u32, u32)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.forward_first_out(), self.cch.forward_head()),
            &self.upward_intervals,
            &self.upward_bounds,
        )
    }

    pub fn backward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<u32>, &Vec<(u32, u32)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.backward_first_out(), self.cch.backward_head()),
            &self.downward_intervals,
            &self.downward_bounds,
        )
    }
}

/*fn prepare_weights(
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
}*/

/*fn extract_interval_minima(weights: &Vec<Vec<TTFPoint>>, num_intervals: u32) -> Vec<u32> {
    let interval_length = MAX_BUCKETS / num_intervals;

    weights
        .iter()
        .flat_map(|ttf| {
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
        .collect::<Vec<u32>>()
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
}*/

fn build_customized_graph(
    cch: &CCH,
    upward_intervals: &mut Vec<Vec<u32>>,
    upward_bounds: &Vec<(u32, u32)>,
    downward_intervals: &mut Vec<Vec<u32>>,
    downward_bounds: &Vec<(u32, u32)>,
    num_intervals: u32,
) -> (DirectedCCH, Vec<u32>, Vec<u32>, Vec<(u32, u32)>, Vec<(u32, u32)>) {
    let m = cch.num_arcs();
    let n = cch.num_nodes();

    let forward = UnweightedFirstOutGraph::new(cch.forward_first_out(), cch.forward_head());
    let backward = UnweightedFirstOutGraph::new(cch.backward_first_out(), cch.backward_head());

    let mut forward_first_out = Vec::with_capacity(cch.first_out.len());
    forward_first_out.push(0);
    let mut forward_head = Vec::with_capacity(m);
    let mut forward_bounds = Vec::with_capacity(m);
    let mut forward_cch_edge_to_orig_arc = Vec::with_capacity(m);
    println!("Allocated forward structs");

    let mut backward_first_out = Vec::with_capacity(cch.first_out.len());
    backward_first_out.push(0);
    let mut backward_head = Vec::with_capacity(m);

    let mut backward_bounds = Vec::with_capacity(m);
    let mut backward_cch_edge_to_orig_arc = Vec::with_capacity(m);
    println!("Allocated backward structs");

    // count how many edges will survive, allocate required memory
    let upward_count = upward_intervals.iter().filter(|v| !v.is_empty()).count();
    let downward_count = downward_intervals.iter().filter(|v| !v.is_empty()).count();

    let mut forward_weights = vec![0; upward_count * num_intervals as usize];
    let mut backward_weights = vec![0; downward_count * num_intervals as usize];
    println!("Allocated weights");

    let mut forward_edge_counter = 0;
    let mut backward_edge_counter = 0;

    for node in 0..n as NodeId {
        let edge_ids = cch.neighbor_edge_indices_usize(node);

        for ((((NodeIdT(next_node), _), forward_orig_arcs), intervals), bounds) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&forward, node)
            .zip(edge_ids.clone().into_iter().map(|e| &cch.forward_cch_edge_to_orig_arc[e]))
            .zip(&mut upward_intervals[edge_ids.clone()])
            .zip(&upward_bounds[edge_ids.clone()])
        {
            // pruning: ignore edge if lower bound exceeds customized upper bound
            if !intervals.is_empty() {
                forward_head.push(next_node);
                for interval_idx in 0..intervals.len() {
                    forward_weights[interval_idx * upward_count + forward_edge_counter as usize] = intervals[interval_idx];
                }
                forward_bounds.push(*bounds);
                forward_cch_edge_to_orig_arc.push(forward_orig_arcs.to_vec());
                forward_edge_counter += 1;

                // reduce memory consumption by removing unnecessary content
                intervals.clear();
                intervals.shrink_to(0);
            }
        }

        for ((((NodeIdT(next_node), _), backward_orig_arcs), intervals), bounds) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&backward, node)
            .zip(edge_ids.clone().into_iter().map(|e| &cch.backward_cch_edge_to_orig_arc[e]))
            .zip(&mut downward_intervals[edge_ids.clone()])
            .zip(&downward_bounds[edge_ids.clone()])
        {
            if !intervals.is_empty() {
                backward_head.push(next_node);

                for interval_idx in 0..intervals.len() {
                    backward_weights[interval_idx * downward_count + backward_edge_counter as usize] = intervals[interval_idx];
                }
                backward_bounds.push(*bounds);
                backward_cch_edge_to_orig_arc.push(backward_orig_arcs.to_vec());
                backward_edge_counter += 1;

                // reduce memory consumption by removing unnecessary content
                intervals.clear();
                intervals.shrink_to(0);
            }
        }
        forward_first_out.push(forward_edge_counter);
        backward_first_out.push(backward_edge_counter);
    }

    let forward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&forward_first_out[..], &forward_head[..]));
    let backward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&backward_first_out[..], &backward_head[..]));
    let node_order = cch.node_order.clone();
    let elimination_tree = cch.elimination_tree.clone();

    let cch = DirectedCCH::new(
        forward_first_out,
        forward_head,
        backward_first_out,
        backward_head,
        node_order,
        forward_cch_edge_to_orig_arc,
        backward_cch_edge_to_orig_arc,
        elimination_tree,
        forward_inverted,
        backward_inverted,
    );

    (cch, forward_weights, backward_weights, forward_bounds, backward_bounds)
}

/*fn empty_ttf() -> Vec<TTFPoint> {
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
}*/

/*pub fn reorder_edge_intervals(intervals: &Vec<u32>, num_edges: usize, num_intervals: u32) -> Vec<u32> {
    let mut ret = vec![0; intervals.len()];

    for edge_id in 0..num_edges {
        for interval_id in 0..num_intervals as usize {
            ret[interval_id * num_edges + edge_id] = intervals[edge_id * num_intervals as usize + interval_id];
        }
    }
    ret
}*/
