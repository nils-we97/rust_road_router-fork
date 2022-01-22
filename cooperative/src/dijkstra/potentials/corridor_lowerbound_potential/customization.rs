use crate::dijkstra::potentials::cch_lower_upper::bounded_potential::BoundedLowerUpperPotentialContext;
use crate::dijkstra::potentials::corridor_lowerbound_potential::customization_catchup::customize_td_graph;
use crate::dijkstra::potentials::corridor_lowerbound_potential::shortcut::ShortcutWrapper;
use crate::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotentialContext;
use crate::graph::capacity_graph::CapacityGraph;
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

pub struct CustomizedCorridorLowerbound {
    pub cch: DirectedCCH,
    pub upward_intervals: Vec<u32>,
    pub downward_intervals: Vec<u32>,
    pub upward_bounds: Vec<(u32, u32)>,
    pub downward_bounds: Vec<(u32, u32)>,
    pub num_intervals: u32,
    pub potential_context: CorridorLowerboundPotentialContext,
    pub corridor_context: BoundedLowerUpperPotentialContext,
}

impl CustomizedCorridorLowerbound {
    pub fn new_from_capacity(cch: &CCH, graph: &CapacityGraph, num_intervals: u32) -> Self {
        // basic workaround: convert to TD-Graph, then run PTV customization
        let mut first_ipp_of_arc = vec![0];
        let mut departure = Vec::new();
        let mut travel_time = Vec::new();

        graph.departure().iter().zip(graph.travel_time().iter()).for_each(|(dep, tt)| {
            // constant edges only have one entry (0, tt) whereas non-constant edge functions are periodic
            if tt.iter().all(|&val| val == tt[0]) {
                first_ipp_of_arc.push(*first_ipp_of_arc.last().unwrap() + 1);
                departure.push(dep[0]);
                travel_time.push(tt[0]);
            } else {
                first_ipp_of_arc.push(*first_ipp_of_arc.last().unwrap() + dep.len() as u32);
                departure.extend_from_slice(dep);
                travel_time.extend_from_slice(tt);
            }
        });

        let td_graph = TDGraph::new(graph.first_out().to_vec(), graph.head().to_vec(), first_ipp_of_arc, departure, travel_time);

        Self::run_customization(cch, &td_graph, num_intervals)
    }

    pub fn new_from_ptv(cch: &CCH, graph: &TDGraph, num_intervals: u32) -> Self {
        Self::run_customization(cch, graph, num_intervals)
    }

    fn run_customization(cch: &CCH, graph: &TDGraph, num_intervals: u32) -> Self {
        debug_assert!(MAX_BUCKETS % num_intervals == 0);

        let ((mut upward_weights, mut downward_weights), time) = measure(|| customize_td_graph(cch, graph, num_intervals));
        println!("Interval Minima Customization took {} ms", time.as_secs_f64() * 1000.0);

        // extract relevant data, scale upper bounds
        let (mut upward_intervals, upward_bounds, num_removed_edges) = extract_intervals_and_bounds(&mut upward_weights);
        println!(
            "Reduction in upward direction: Removed {} of {} edges.",
            num_removed_edges,
            upward_weights.len()
        );

        let (mut downward_intervals, downward_bounds, num_removed_edges) = extract_intervals_and_bounds(&mut downward_weights);
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

        let num_nodes = cch.num_nodes();
        Self {
            cch,
            upward_intervals,
            downward_intervals,
            upward_bounds,
            downward_bounds,
            num_intervals,
            potential_context: CorridorLowerboundPotentialContext::new(num_nodes),
            corridor_context: BoundedLowerUpperPotentialContext::new(num_nodes),
        }
    }

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

fn extract_intervals_and_bounds(weights: &mut Vec<ShortcutWrapper>) -> (Vec<Vec<u32>>, Vec<(u32, u32)>, u32) {
    let mut num_removed_edges = 0;
    let (intervals, bounds) = weights
        .iter_mut()
        .map(|wrapper| {
            let ret = if wrapper.shortcut.required && wrapper.bounds.0 <= wrapper.bounds.1 {
                (wrapper.interval_minima.clone(), wrapper.bounds)
            } else {
                num_removed_edges += 1;
                (vec![], (INFINITY, INFINITY))
            };

            wrapper.interval_minima = vec![];
            ret
        })
        .unzip();

    (intervals, bounds, num_removed_edges)
}

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
