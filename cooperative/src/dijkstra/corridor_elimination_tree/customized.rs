use crate::dijkstra::corridor_elimination_tree::parallelization::{SeparatorBasedParallelCustomization, SeparatorBasedPerfectParallelCustomization};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::{DirectedCCH, CCH, CCHT};
use rust_road_router::datastr::graph::{
    BuildReversed, EdgeId, EdgeIdT, FirstOutGraph, Graph, Link, LinkIterable, NodeId, NodeIdT, Reversed, ReversedGraphWithEdgeIds, UnweightedFirstOutGraph,
    Weight, INFINITY,
};
use rust_road_router::report::{report_time, report_time_with_key};
use rust_road_router::util::in_range_option::InRangeOption;
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::min;
use std::ops::Range;

// One mapping of node id to weight for each thread during the scope of the customization.
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<(Weight, Weight)>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<(Weight, Weight)>>);
scoped_thread_local!(static PERFECT_WORKSPACE: RefCell<Vec<InRangeOption<EdgeId>>>);

pub struct CustomizedUpperLower {
    pub cch: DirectedCCH,
    pub upward: Vec<(Weight, Weight)>,
    pub downward: Vec<(Weight, Weight)>,
    pub _phantom: std::marker::PhantomData<DirectedCCH>,
}

impl CustomizedUpperLower {
    pub fn new(cch: &CCH, travel_times: &Vec<Vec<Weight>>) -> Self {
        let m = cch.num_arcs();

        let (lower_bound, upper_bound) = travel_times
            .iter()
            .map(|times| (times.iter().max().unwrap().clone(), times.iter().min().unwrap().clone()))
            .unzip();

        // these will contain our customized shortcuts
        let mut upward_weights = vec![(INFINITY, INFINITY); m];
        let mut downward_weights = vec![(INFINITY, INFINITY); m];

        // initialize upward and downward weights with correct lower/upper bound
        prepare_weights(cch, &mut upward_weights, &mut downward_weights, &lower_bound, &upper_bound);

        // run basic customization
        customize_basic(cch, &mut upward_weights, &mut downward_weights);

        // run perfect customization
        let directed_cch = customize_perfect(cch, &mut upward_weights, &mut downward_weights);

        Self {
            cch: directed_cch,
            upward: upward_weights,
            downward: downward_weights,
            _phantom: Default::default(),
        }
    }

    pub fn forward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<(Weight, Weight)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.forward_first_out(), self.cch.forward_head()),
            &self.upward,
        )
    }

    pub fn backward_graph(&self) -> (UnweightedFirstOutGraph<&[EdgeId], &[NodeId]>, &Vec<(Weight, Weight)>) {
        (
            UnweightedFirstOutGraph::new(self.cch.backward_first_out(), self.cch.backward_head()),
            &self.downward,
        )
    }
}

// subroutines
fn prepare_weights(cch: &CCH, upward_weights: &mut Vec<(u32, u32)>, downward_weights: &mut Vec<(u32, u32)>, lower_bound: &Vec<u32>, upper_bound: &Vec<u32>) {
    report_time("Apply weights", || {
        upward_weights
            .par_iter_mut()
            .zip(downward_weights.par_iter_mut())
            .zip(cch.cch_edge_to_orig_arc.par_iter())
            .for_each(|(((up_lower, up_upper), (down_lower, down_upper)), (up_arcs, down_arcs))| {
                for &EdgeIdT(up_arc) in up_arcs {
                    *up_lower = min(*up_lower, lower_bound[up_arc as usize]);
                    *up_upper = min(*up_upper, upper_bound[up_arc as usize]);
                }
                for &EdgeIdT(down_arc) in down_arcs {
                    *down_lower = min(*down_lower, lower_bound[down_arc as usize]);
                    *down_upper = min(*down_upper, upper_bound[down_arc as usize]);
                }
            });
    });
}

fn customize_basic(cch: &CCH, upward_weights: &mut Vec<(Weight, Weight)>, downward_weights: &mut Vec<(Weight, Weight)>) {
    let n = cch.num_nodes() as NodeId;

    let customize = |nodes: Range<usize>, offset: usize, upward_weights: &mut [(Weight, Weight)], downward_weights: &mut [(Weight, Weight)]| {
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
                        node_incoming_weights[node as usize] = (down.0, down.1);
                        node_outgoing_weights[node as usize] = (up.0, up.1);
                    }

                    for (NodeIdT(low_node), Reversed(EdgeIdT(first_edge_id))) in cch.inverted.link_iter(current_node) {
                        let first_down_weight: &(Weight, Weight) = &downward_weights[first_edge_id as usize - offset];
                        let first_up_weight: &(Weight, Weight) = &upward_weights[first_edge_id as usize - offset];
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
                            relax.0 = std::cmp::min(relax.0, upward_weight.0 + first_down_weight.0);
                            relax.1 = std::cmp::min(relax.1, upward_weight.1 + first_down_weight.1);
                            let relax = unsafe { node_incoming_weights.get_unchecked_mut(node as usize) };
                            relax.0 = std::cmp::min(relax.0, downward_weight.0 + first_up_weight.0);
                            relax.1 = std::cmp::min(relax.1, downward_weight.1 + first_up_weight.1);
                        }
                    }

                    for (((node, down), up), _edge_id) in cch
                        .neighbor_iter(current_node)
                        .zip(&mut downward_weights[edges.clone()])
                        .zip(&mut upward_weights[edges.clone()])
                        .zip(edges)
                    {
                        down.0 = node_incoming_weights[node as usize].0;
                        down.1 = node_incoming_weights[node as usize].1;
                        up.0 = node_outgoing_weights[node as usize].0;
                        up.1 = node_outgoing_weights[node as usize].1;
                    }
                }
            });
        });
    };

    // setup customization for parallization
    let customization = SeparatorBasedParallelCustomization::new(cch, customize, customize);

    // execute customization
    report_time_with_key("CCH Customization", "basic_customization", || {
        customization.customize(upward_weights, downward_weights, |cb| {
            // create workspace vectors for the scope of the customization
            UPWARD_WORKSPACE.set(&RefCell::new(vec![(INFINITY, INFINITY); n as usize]), || {
                DOWNWARD_WORKSPACE.set(&RefCell::new(vec![(INFINITY, INFINITY); n as usize]), || cb());
            });
            // everything will be dropped here
        });
    });
}

// perfect customization
fn customize_perfect(cch: &CCH, upward_weights: &mut Vec<(Weight, Weight)>, downward_weights: &mut Vec<(Weight, Weight)>) -> DirectedCCH {
    let n = cch.num_nodes();
    let m = cch.num_arcs();
    // Routine for perfect precustomization.
    // The interface is similar to the one for the basic customization, but we need access to nonconsecutive ranges of edges,
    // so we can't use slices. Thus, we just take a mutable pointer to the shortcut vecs.
    // The logic of the perfect customization based on separators guarantees, that we will never concurrently modify
    // the same shortcuts, but so far I haven't found a way to express that in safe rust.
    let customize_perfect = |nodes: Range<usize>, upward: *mut (Weight, Weight), downward: *mut (Weight, Weight)| {
        PERFECT_WORKSPACE.with(|node_edge_ids| {
            let mut node_edge_ids = node_edge_ids.borrow_mut();

            // processing nodes in reverse order
            for current_node in nodes.rev() {
                let current_node = current_node as NodeId;
                // store mapping of head node to corresponding outgoing edge id
                for (node, edge_id) in cch.neighbor_iter(current_node).zip(cch.neighbor_edge_indices(current_node)) {
                    node_edge_ids[node as usize] = InRangeOption::new(Some(edge_id));
                }

                for (node, edge_id) in cch.neighbor_iter(current_node).zip(cch.neighbor_edge_indices(current_node)) {
                    let shortcut_edge_ids = cch.neighbor_edge_indices(node);
                    for (target, shortcut_edge_id) in cch.neighbor_iter(node).zip(shortcut_edge_ids) {
                        if let Some(other_edge_id) = node_edge_ids[target as usize].value() {
                            // Here we have both an intermediate and an upper triangle
                            // depending on which edge we take as the base
                            // Relax all them.
                            unsafe {
                                (*upward.add(other_edge_id as usize)).1 = min(
                                    (*upward.add(other_edge_id as usize)).1,
                                    (*upward.add(edge_id as usize)).1 + (*upward.add(shortcut_edge_id as usize)).1,
                                );
                                (*upward.add(edge_id as usize)).1 = min(
                                    (*upward.add(edge_id as usize)).1,
                                    (*upward.add(other_edge_id as usize)).1 + (*downward.add(shortcut_edge_id as usize)).1,
                                );
                                (*downward.add(other_edge_id as usize)).1 = min(
                                    (*downward.add(other_edge_id as usize)).1,
                                    (*downward.add(edge_id as usize)).1 + (*downward.add(shortcut_edge_id as usize)).1,
                                );
                                (*downward.add(edge_id as usize)).1 = min(
                                    (*downward.add(edge_id as usize)).1,
                                    (*downward.add(other_edge_id as usize)).1 + (*upward.add(shortcut_edge_id as usize)).1,
                                );
                            }
                        }
                    }
                }

                // reset the mapping
                for node in cch.neighbor_iter(current_node) {
                    node_edge_ids[node as usize] = InRangeOption::new(None);
                }
            }
        });
    };

    let static_perfect_customization = SeparatorBasedPerfectParallelCustomization::new(cch, customize_perfect, customize_perfect);

    let upward_orig = upward_weights.clone();
    let downward_orig = downward_weights.clone();

    report_time_with_key("CCH Perfect Customization", "perfect_customization", || {
        static_perfect_customization.customize(upward_weights, downward_weights, |cb| {
            PERFECT_WORKSPACE.set(&RefCell::new(vec![InRangeOption::new(None); n as usize]), || cb());
        });
    });

    report_time_with_key("Build Perfect Customized Graph", "graph_build", || {
        let forward = FirstOutGraph::new(
            cch.forward_first_out(),
            cch.forward_head(),
            upward_weights.iter().map(|&(lower, _)| lower).collect::<Vec<Weight>>(),
        );
        let backward = FirstOutGraph::new(
            cch.backward_first_out(),
            cch.backward_head(),
            downward_weights.iter().map(|&(lower, _)| lower).collect::<Vec<Weight>>(),
        );

        let mut forward_first_out = Vec::with_capacity(cch.first_out.len());
        forward_first_out.push(0);
        let mut forward_head = Vec::with_capacity(m);
        let mut forward_weight = Vec::with_capacity(m);
        let mut forward_cch_edge_to_orig_arc = Vec::with_capacity(m);

        let mut backward_first_out = Vec::with_capacity(cch.first_out.len());
        backward_first_out.push(0);
        let mut backward_head = Vec::with_capacity(m);
        let mut backward_weight = Vec::with_capacity(m);
        let mut backward_cch_edge_to_orig_arc = Vec::with_capacity(m);

        let mut forward_edge_counter = 0;
        let mut backward_edge_counter = 0;

        for node in 0..n as NodeId {
            let edge_ids = cch.neighbor_edge_indices_usize(node);
            let orig_arcs = &cch.cch_edge_to_orig_arc[edge_ids.clone()];
            for ((link, (forward_orig_arcs, _)), &customized_weight) in LinkIterable::<Link>::link_iter(&forward, node)
                .zip(orig_arcs.iter())
                .zip(&upward_orig[edge_ids.clone()])
            {
                // lowerbound must be smaller than upperbound
                if link.weight < INFINITY && !(link.weight <= customized_weight.1) {
                    forward_head.push(link.node);
                    forward_weight.push(link.weight);
                    forward_cch_edge_to_orig_arc.push(forward_orig_arcs.clone());
                    forward_edge_counter += 1;
                }
            }
            for ((link, (_, backward_orig_arcs)), &customized_weight) in LinkIterable::<Link>::link_iter(&backward, node)
                .zip(orig_arcs.iter())
                .zip(&downward_orig[edge_ids.clone()])
            {
                if link.weight < INFINITY && !(link.weight <= customized_weight.1) {
                    backward_head.push(link.node);
                    backward_weight.push(link.weight);
                    backward_cch_edge_to_orig_arc.push(backward_orig_arcs.clone());
                    backward_edge_counter += 1;
                }
            }
            forward_first_out.push(forward_edge_counter);
            backward_first_out.push(backward_edge_counter);
        }

        let forward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&forward_first_out[..], &forward_head[..]));
        let backward_inverted = ReversedGraphWithEdgeIds::reversed(&UnweightedFirstOutGraph::new(&backward_first_out[..], &backward_head[..]));
        let node_order = cch.node_order.clone();
        let elimination_tree = cch.elimination_tree.clone();

        DirectedCCH::new(
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
        )
    })
}
