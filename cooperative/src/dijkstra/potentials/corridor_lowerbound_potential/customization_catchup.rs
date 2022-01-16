use crate::dijkstra::potentials::cch_parallelization_util::{
    ForEachIter, ParIter, SeparatorBasedParallelCustomization, SeparatorBasedPerfectParallelCustomization, SeqIter,
};
use crate::dijkstra::potentials::corridor_lowerbound_potential::shortcut::{PartialShortcutWrapperGraph, ShortcutWrapper};
use crate::dijkstra::potentials::{convert_timestamp_f64_to_u32, convert_timestamp_u32_to_f64};
use crate::graph::MAX_BUCKETS;
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::floating_time_dependent::shortcut::Sources;
use rust_road_router::datastr::graph::floating_time_dependent::shortcut_source::ShortcutSource;
use rust_road_router::datastr::graph::floating_time_dependent::{
    FlWeight, MergeBuffers, PeriodicATTF, PeriodicPiecewiseLinearFunction, Shortcut, TDGraph, TTFPoint, Timestamp, PLF,
};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, Reversed, INFINITY};
use rust_road_router::report;
use rust_road_router::report::*;
use rust_road_router::util::in_range_option::InRangeOption;
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::{max, min, Ord};
use std::ops::Range;

// Reusable buffers for main CATCHUp customization, to reduce allocations
scoped_thread_local!(static MERGE_BUFFERS: RefCell<MergeBuffers>);
// Workspaces for static CATCHUp precustomization - similar to regular static customization - see parent module
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<(FlWeight, FlWeight)>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<(FlWeight, FlWeight)>>);
// Workspace for perfect static CATCHUp precustomization - here we just need one for both directions
// because we map to the edge id instead of the values.
scoped_thread_local!(static PERFECT_WORKSPACE: RefCell<Vec<InRangeOption<EdgeId>>>);

pub fn customize_td_graph(cch: &CCH, metric: &TDGraph, num_intervals: u32) -> (Vec<ShortcutWrapper>, Vec<ShortcutWrapper>) {
    report!("algo", "Floating TDCCH Customization");

    let n = (cch.first_out.len() - 1) as NodeId;
    let m = cch.head.len();

    // these will contain our customized shortcuts
    let mut upward: Vec<_> = std::iter::repeat_with(|| ShortcutWrapper::empty(Shortcut::new(None, metric))).take(m).collect();
    let mut downward: Vec<_> = std::iter::repeat_with(|| ShortcutWrapper::empty(Shortcut::new(None, metric))).take(m).collect();

    // start with respecting - set shortcuts to respective original edge.
    let subctxt = push_context("weight_applying".to_string());
    report_time("TD-CCH apply weights", || {
        upward
            .par_iter_mut()
            .zip(cch.forward_cch_edge_to_orig_arc.par_iter())
            .for_each(|(up_weight, up_arcs)| {
                assert!(up_arcs.len() <= 1);
                for &EdgeIdT(up_arc) in up_arcs {
                    *up_weight = ShortcutWrapper::empty(Shortcut::new(Some(up_arc), metric));
                }
            });

        downward
            .par_iter_mut()
            .zip(cch.backward_cch_edge_to_orig_arc.par_iter())
            .for_each(|(down_weight, down_arcs)| {
                assert!(down_arcs.len() <= 1);
                for &EdgeIdT(down_arc) in down_arcs {
                    *down_weight = ShortcutWrapper::empty(Shortcut::new(Some(down_arc), metric));
                }
            });
    });
    drop(subctxt);

    // This is the routine for basic static customization with just the upper and lower bounds.
    // It runs completely analogue the standard customization algorithm.
    let customize = |nodes: Range<usize>, offset, upward_weights: &mut [ShortcutWrapper], downward_weights: &mut [ShortcutWrapper]| {
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
                        node_incoming_weights[node as usize] = (down.shortcut.lower_bound, down.shortcut.upper_bound);
                        node_outgoing_weights[node as usize] = (up.shortcut.lower_bound, up.shortcut.upper_bound);
                    }

                    for (NodeIdT(low_node), Reversed(EdgeIdT(first_edge_id))) in cch.inverted.link_iter(current_node) {
                        let first_down_weight: &ShortcutWrapper = &downward_weights[first_edge_id as usize - offset];
                        let first_up_weight: &ShortcutWrapper = &upward_weights[first_edge_id as usize - offset];
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
                            relax.0 = std::cmp::min(relax.0, upward_weight.shortcut.lower_bound + first_down_weight.shortcut.lower_bound);
                            relax.1 = std::cmp::min(relax.1, upward_weight.shortcut.upper_bound + first_down_weight.shortcut.upper_bound);
                            let relax = unsafe { node_incoming_weights.get_unchecked_mut(node as usize) };
                            relax.0 = std::cmp::min(relax.0, downward_weight.shortcut.lower_bound + first_up_weight.shortcut.lower_bound);
                            relax.1 = std::cmp::min(relax.1, downward_weight.shortcut.upper_bound + first_up_weight.shortcut.upper_bound);
                        }
                    }

                    for (((node, down), up), _edge_id) in cch
                        .neighbor_iter(current_node)
                        .zip(&mut downward_weights[edges.clone()])
                        .zip(&mut upward_weights[edges.clone()])
                        .zip(edges)
                    {
                        down.shortcut.lower_bound = node_incoming_weights[node as usize].0;
                        down.shortcut.upper_bound = node_incoming_weights[node as usize].1;
                        up.shortcut.lower_bound = node_outgoing_weights[node as usize].0;
                        up.shortcut.upper_bound = node_outgoing_weights[node as usize].1;
                        down.shortcut.update_is_constant();
                        up.shortcut.update_is_constant();
                    }
                }
            });
        });
    };

    // Routine for CATCHUp perfect precustomization on the bounds.
    // The interface is similar to the one for the basic customization, but we need access to nonconsecutive ranges of edges,
    // so we can't use slices. Thus, we just take a mutable pointer to the shortcut vecs.
    // The logic of the perfect customization based on separators guarantees, that we will never concurrently modify
    // the same shortcuts, but so far I haven't found a way to express that in safe rust.
    let customize_perfect = |nodes: Range<usize>, upward: *mut ShortcutWrapper, downward: *mut ShortcutWrapper| {
        PERFECT_WORKSPACE.with(|node_edge_ids| {
            let mut node_edge_ids = node_edge_ids.borrow_mut();

            // processing nodes in reverse order
            for current_node in nodes.rev() {
                let current_node = current_node as NodeId;
                // store mapping of head node to corresponding outgoing edge id
                for (node, edge_id) in cch.neighbor_iter(current_node).zip(cch.neighbor_edge_indices(current_node)) {
                    node_edge_ids[node as usize] = InRangeOption::some(edge_id);
                }

                for (node, edge_id) in cch.neighbor_iter(current_node).zip(cch.neighbor_edge_indices(current_node)) {
                    let shortcut_edge_ids = cch.neighbor_edge_indices(node);
                    for (target, shortcut_edge_id) in cch.neighbor_iter(node).zip(shortcut_edge_ids) {
                        if let Some(other_edge_id) = node_edge_ids[target as usize].value() {
                            // Here we have both an intermediate and an upper triangle
                            // depending on which edge we take as the base
                            // Relax all them.
                            unsafe {
                                (*upward.add(other_edge_id as usize)).shortcut.upper_bound = min(
                                    (*upward.add(other_edge_id as usize)).shortcut.upper_bound,
                                    (*upward.add(edge_id as usize)).shortcut.upper_bound + (*upward.add(shortcut_edge_id as usize)).shortcut.upper_bound,
                                );
                                (*upward.add(other_edge_id as usize)).shortcut.lower_bound = min(
                                    (*upward.add(other_edge_id as usize)).shortcut.lower_bound,
                                    (*upward.add(edge_id as usize)).shortcut.lower_bound + (*upward.add(shortcut_edge_id as usize)).shortcut.lower_bound,
                                );

                                (*upward.add(edge_id as usize)).shortcut.upper_bound = min(
                                    (*upward.add(edge_id as usize)).shortcut.upper_bound,
                                    (*upward.add(other_edge_id as usize)).shortcut.upper_bound
                                        + (*downward.add(shortcut_edge_id as usize)).shortcut.upper_bound,
                                );
                                (*upward.add(edge_id as usize)).shortcut.lower_bound = min(
                                    (*upward.add(edge_id as usize)).shortcut.lower_bound,
                                    (*upward.add(other_edge_id as usize)).shortcut.lower_bound
                                        + (*downward.add(shortcut_edge_id as usize)).shortcut.lower_bound,
                                );

                                (*downward.add(other_edge_id as usize)).shortcut.upper_bound = min(
                                    (*downward.add(other_edge_id as usize)).shortcut.upper_bound,
                                    (*downward.add(edge_id as usize)).shortcut.upper_bound + (*downward.add(shortcut_edge_id as usize)).shortcut.upper_bound,
                                );
                                (*downward.add(other_edge_id as usize)).shortcut.lower_bound = min(
                                    (*downward.add(other_edge_id as usize)).shortcut.lower_bound,
                                    (*downward.add(edge_id as usize)).shortcut.lower_bound + (*downward.add(shortcut_edge_id as usize)).shortcut.lower_bound,
                                );

                                (*downward.add(edge_id as usize)).shortcut.upper_bound = min(
                                    (*downward.add(edge_id as usize)).shortcut.upper_bound,
                                    (*downward.add(other_edge_id as usize)).shortcut.upper_bound
                                        + (*upward.add(shortcut_edge_id as usize)).shortcut.upper_bound,
                                );
                                (*downward.add(edge_id as usize)).shortcut.lower_bound = min(
                                    (*downward.add(edge_id as usize)).shortcut.lower_bound,
                                    (*downward.add(other_edge_id as usize)).shortcut.lower_bound
                                        + (*upward.add(shortcut_edge_id as usize)).shortcut.lower_bound,
                                );
                            }
                        }
                    }
                }

                // reset the mapping
                for node in cch.neighbor_iter(current_node) {
                    node_edge_ids[node as usize] = InRangeOption::NONE;
                }
            }
        });
    };

    // parallelize precusotmization
    let static_customization = SeparatorBasedParallelCustomization::new(cch, customize, customize);
    let static_perfect_customization = SeparatorBasedPerfectParallelCustomization::new(cch, customize_perfect, customize_perfect);

    // routine to disable shortcuts for which the perfect precustomization determined them to be irrelevant
    let disable_dominated = |(wrapper, &lower_bound): (&mut ShortcutWrapper, &FlWeight)| {
        // shortcut contains shortest path length, lower bound the length of the specific path represented by the shortcut (not necessarily the shortest)
        if wrapper.shortcut.upper_bound.fuzzy_lt(lower_bound) {
            wrapper.shortcut.required = false;
            wrapper.shortcut.lower_bound = FlWeight::INFINITY;
            wrapper.shortcut.upper_bound = FlWeight::INFINITY;
        } else {
            // reset shortcut lower bound from path to actual shortcut bound
            wrapper.shortcut.lower_bound = lower_bound;
        }
    };

    // precustomization
    let _subctxt = push_context("precustomization".to_string());
    report_time("TD-CCH Pre-Customization", || {
        static_customization.customize(&mut upward, &mut downward, |cb| {
            UPWARD_WORKSPACE.set(&RefCell::new(vec![(FlWeight::INFINITY, FlWeight::INFINITY); n as usize]), || {
                DOWNWARD_WORKSPACE.set(&RefCell::new(vec![(FlWeight::INFINITY, FlWeight::INFINITY); n as usize]), cb);
            });
        });

        let upward_preliminary_bounds: Vec<_> = upward.iter().map(|s| s.shortcut.lower_bound).collect();
        let downward_preliminary_bounds: Vec<_> = downward.iter().map(|s| s.shortcut.lower_bound).collect();

        static_perfect_customization.customize(&mut upward, &mut downward, |cb| {
            PERFECT_WORKSPACE.set(&RefCell::new(vec![InRangeOption::NONE; n as usize]), cb);
        });

        upward.par_iter_mut().zip(upward_preliminary_bounds.par_iter()).for_each(disable_dominated);
        downward.par_iter_mut().zip(downward_preliminary_bounds.par_iter()).for_each(disable_dominated);
    });

    // block for main CATCHUp customization
    {
        let subctxt = push_context("main".to_string());

        // use separator based parallelization
        let customization = SeparatorBasedParallelCustomization::new(
            cch,
            // routines created in this function
            // we customize many cells in parallel - so iterate over triangles sequentially
            create_customization_fn(&cch, metric, SeqIter(&cch), num_intervals),
            // the final separator can only be customized, once everything else is done, but it still takes up a significant amount of time
            // But we can still parallelize the processing of edges from one node within this separator.
            create_customization_fn(&cch, metric, ParIter(&cch), num_intervals),
        );

        report_time("TD-CCH Customization", || {
            // execute main customization
            customization.customize(&mut upward, &mut downward, |cb| {
                MERGE_BUFFERS.set(&RefCell::new(MergeBuffers::new()), || {
                    cb();
                });
            });
        });

        drop(subctxt);
    }

    // post-customization
    // do perfect bound based customization again, because we now have better bounds and can get rid of some additional shortcuts
    let _subctxt = push_context("postcustomization".to_string());
    report_time("TD-CCH Post-Customization", || {
        let upward_preliminary_bounds: Vec<_> = upward.iter().map(|s| s.shortcut.lower_bound).collect();
        let downward_preliminary_bounds: Vec<_> = downward.iter().map(|s| s.shortcut.lower_bound).collect();

        static_perfect_customization.customize(&mut upward, &mut downward, |cb| {
            PERFECT_WORKSPACE.set(&RefCell::new(vec![InRangeOption::NONE; n as usize]), || cb());
        });

        // routine to disable shortcuts for which the perfect precustomization determined them to be irrelevant
        let disable_dominated = |(wrapper, &lower_bound): (&mut ShortcutWrapper, &FlWeight)| {
            // shortcut contains shortest path length, lower bound the length of the specific path represented by the shortcut (not necessarily the shortest)
            if wrapper.shortcut.upper_bound.fuzzy_lt(lower_bound) {
                wrapper.shortcut.required = false;
            }
            // reset shortcut lower bound from path to actual shortcut bound
            wrapper.shortcut.lower_bound = lower_bound;
        };

        upward.par_iter_mut().zip(upward_preliminary_bounds.par_iter()).for_each(disable_dominated);
        downward.par_iter_mut().zip(downward_preliminary_bounds.par_iter()).for_each(disable_dominated);

        for current_node in 0..n {
            let (upward_below, upward_above) = upward.split_at_mut(cch.first_out[current_node as usize] as usize);
            let upward_active = &mut upward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let (downward_below, downward_above) = downward.split_at_mut(cch.first_out[current_node as usize] as usize);
            let downward_active = &mut downward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let shortcut_graph = PartialShortcutWrapperGraph::new(metric, upward_below, downward_below, 0);

            for wrapper in &mut upward_active[..] {
                wrapper.disable_if_unnecessary(&shortcut_graph);
            }

            for wrapper in &mut downward_active[..] {
                wrapper.disable_if_unnecessary(&shortcut_graph);
            }
        }

        for current_node in (0..n).rev() {
            let (upward_below, upward_above) = upward.split_at_mut(cch.first_out[current_node as usize] as usize);
            let upward_active = &mut upward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let (downward_below, downward_above) = downward.split_at_mut(cch.first_out[current_node as usize] as usize);
            let downward_active = &mut downward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];

            for wrapper in &mut upward_active[..] {
                wrapper.reenable_required(downward_below, upward_below);
            }

            for wrapper in &mut downward_active[..] {
                wrapper.reenable_required(downward_below, upward_below);
            }
        }

        upward.par_iter_mut().for_each(|s| {
            if !s.shortcut.required {
                s.shortcut.lower_bound = FlWeight::INFINITY;
                s.shortcut.upper_bound = FlWeight::INFINITY;
            }
        });
        downward.par_iter_mut().for_each(|s| {
            if !s.shortcut.required {
                s.shortcut.lower_bound = FlWeight::INFINITY;
                s.shortcut.upper_bound = FlWeight::INFINITY;
            }
        });

        upward
            .par_iter()
            .for_each(|s| debug_assert!(!s.shortcut.required || s.shortcut.lower_bound.fuzzy_lt(FlWeight::INFINITY)));
        downward
            .par_iter()
            .for_each(|s| debug_assert!(!s.shortcut.required || s.shortcut.lower_bound.fuzzy_lt(FlWeight::INFINITY)));
    });

    // adjust upper bounds
    upward.iter_mut().for_each(|wrapper| {
        wrapper.bounds.1 = min(wrapper.bounds.1, convert_timestamp_f64_to_u32(wrapper.shortcut.upper_bound.0));
    });

    downward.iter_mut().for_each(|wrapper| {
        wrapper.bounds.1 = min(wrapper.bounds.1, convert_timestamp_f64_to_u32(wrapper.shortcut.upper_bound.0));
    });

    (upward, downward)
}

// Encapsulates the creation of the CATCHUp main customization lambdas
// The function signature gives us some additional control of lifetimes and stuff
fn create_customization_fn<'s, F: 's>(
    cch: &'s CCH,
    metric: &'s TDGraph,
    merge_iter: F,
    num_intervals: u32,
) -> impl Fn(Range<usize>, usize, &mut [ShortcutWrapper], &mut [ShortcutWrapper]) + 's
where
    for<'p> F: ForEachIter<'p, 's, ShortcutWrapper>,
{
    move |nodes, edge_offset, upward: &mut [ShortcutWrapper], downward: &mut [ShortcutWrapper]| {
        // for all nodes we should currently process
        for current_node in nodes {
            let (upward_below, upward_above) = upward.split_at_mut(cch.first_out[current_node as usize] as usize - edge_offset);
            let upward_active = &mut upward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let (downward_below, downward_above) = downward.split_at_mut(cch.first_out[current_node as usize] as usize - edge_offset);
            let downward_active = &mut downward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let shortcut_graph = PartialShortcutWrapperGraph::new(metric, upward_below, downward_below, edge_offset);

            debug_assert_eq!(upward_active.len(), cch.degree(current_node as NodeId));
            debug_assert_eq!(downward_active.len(), cch.degree(current_node as NodeId));

            // for all outgoing edges - parallel or sequentially, depending on the type of `merge_iter`
            merge_iter.for_each(
                current_node as NodeId,
                upward_active,
                downward_active,
                |(((&node, _), upward_shortcut), downward_shortcut)| {
                    MERGE_BUFFERS.with(|buffers| {
                        let mut buffers = buffers.borrow_mut();

                        // here, we enumerate lower triangles the classic way, as described in the CCH journal
                        // because it is completely dominated by linking and merging.
                        // Also storing the triangles allows us to sort them and process shorter triangles first,
                        // which gives better bounds, which allows skipping unnecessary operations.
                        let mut triangles = Vec::new();

                        // downward edges from both endpoints of the current edge
                        let mut current_iter = cch.inverted.link_iter(current_node as NodeId).peekable();
                        let mut other_iter = cch.inverted.link_iter(node as NodeId).peekable();

                        while let (
                            Some((NodeIdT(lower_from_current), Reversed(EdgeIdT(edge_from_cur_id)))),
                            Some((NodeIdT(lower_from_other), Reversed(EdgeIdT(edge_from_oth_id)))),
                        ) = (current_iter.peek(), other_iter.peek())
                        {
                            debug_assert_eq!(cch.head()[*edge_from_cur_id as usize], current_node as NodeId);
                            debug_assert_eq!(cch.head()[*edge_from_oth_id as usize], node);
                            debug_assert_eq!(cch.edge_id_to_tail(*edge_from_cur_id), *lower_from_current);
                            debug_assert_eq!(cch.edge_id_to_tail(*edge_from_oth_id), *lower_from_other);

                            match lower_from_current.cmp(&lower_from_other) {
                                std::cmp::Ordering::Less => current_iter.next(),
                                std::cmp::Ordering::Greater => other_iter.next(),
                                std::cmp::Ordering::Equal => {
                                    // lower triangle
                                    triangles.push((*edge_from_cur_id, *edge_from_oth_id));

                                    current_iter.next();
                                    other_iter.next()
                                }
                            };
                        }

                        triangles.sort_by_key(|&(down, up)| shortcut_graph.get_incoming(down).lower_bound + shortcut_graph.get_outgoing(up).lower_bound);
                        for &edges in &triangles {
                            // main work happening here
                            upward_shortcut.shortcut.merge(edges, &shortcut_graph, &mut buffers);
                        }
                        upward_shortcut.shortcut.finalize_bounds(&shortcut_graph);

                        triangles.sort_by_key(|&(up, down)| shortcut_graph.get_incoming(down).lower_bound + shortcut_graph.get_outgoing(up).lower_bound);
                        for &(up, down) in &triangles {
                            // and here
                            downward_shortcut.shortcut.merge((down, up), &shortcut_graph, &mut buffers);
                        }
                        downward_shortcut.shortcut.finalize_bounds(&shortcut_graph);
                    });
                },
            );

            // the outgoing edges will never be used again -> transform and free some memory
            for (_, Reversed(EdgeIdT(edge_id))) in cch.inverted.link_iter(current_node as NodeId) {
                if let Some(cache) = &upward[edge_id as usize - edge_offset].shortcut.cache {
                    let ttf = PeriodicATTF::from(cache).bound_plfs().0.to_vec();
                    let (interval_minima, lower_bound, upper_bound) = extract_interval_minima(&ttf, num_intervals);
                    upward[edge_id as usize - edge_offset].interval_minima = interval_minima;
                    upward[edge_id as usize - edge_offset].bounds = (lower_bound, upper_bound);
                } else if let Sources::One(source) = &upward[edge_id as usize - edge_offset].shortcut.sources {
                    if let ShortcutSource::OriginalEdge(id) = ShortcutSource::from(*source) {
                        let ttf = PeriodicATTF::Exact(metric.travel_time_function(id)).bound_plfs().0.to_vec();
                        let (interval_minima, lower_bound, upper_bound) = extract_interval_minima(&ttf, num_intervals);
                        upward[edge_id as usize - edge_offset].interval_minima = interval_minima;
                        upward[edge_id as usize - edge_offset].bounds = (lower_bound, upper_bound);
                    }
                }
                upward[edge_id as usize - edge_offset].shortcut.clear_plf();

                // analogous procedure for downward weights
                if let Some(cache) = &downward[edge_id as usize - edge_offset].shortcut.cache {
                    let ttf = PeriodicATTF::from(cache).bound_plfs().0.to_vec();
                    let (interval_minima, lower_bound, upper_bound) = extract_interval_minima(&ttf, num_intervals);
                    downward[edge_id as usize - edge_offset].interval_minima = interval_minima;
                    downward[edge_id as usize - edge_offset].bounds = (lower_bound, upper_bound);
                } else if let Sources::One(source) = &downward[edge_id as usize - edge_offset].shortcut.sources {
                    if let ShortcutSource::OriginalEdge(id) = ShortcutSource::from(*source) {
                        let ttf = PeriodicATTF::Exact(metric.travel_time_function(id)).bound_plfs().0.to_vec();
                        let (interval_minima, lower_bound, upper_bound) = extract_interval_minima(&ttf, num_intervals);
                        downward[edge_id as usize - edge_offset].interval_minima = interval_minima;
                        downward[edge_id as usize - edge_offset].bounds = (lower_bound, upper_bound);
                    }
                }
                downward[edge_id as usize - edge_offset].shortcut.clear_plf();
            }
        }
    }
}

pub fn convert_to_td_graph(metric: &rust_road_router::datastr::graph::time_dependent::TDGraph) -> TDGraph {
    TDGraph::new(
        metric.first_out().to_vec(),
        metric.head().to_vec(),
        metric.first_ipp_of_arc().to_vec(),
        metric.ipp_departure_time().to_vec(),
        metric.ipp_travel_time().to_vec(),
    )
}

fn extract_interval_minima(ttf: &Vec<TTFPoint>, num_intervals: u32) -> (Vec<u32>, u32, u32) {
    // collect minima within the current interval
    let interval_length = MAX_BUCKETS / num_intervals;
    let mut interval_min = vec![INFINITY; num_intervals as usize];
    let mut bucket_idx = 0;

    let mut global_min = INFINITY;
    let mut global_max = 0;

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

        let val = convert_timestamp_f64_to_u32(point.val.0);
        interval_min[bucket_idx as usize] = min(interval_min[bucket_idx as usize], val);
        global_min = min(global_min, val);
        global_max = max(global_max, val);
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

    (interval_min, global_min, global_max)
}
