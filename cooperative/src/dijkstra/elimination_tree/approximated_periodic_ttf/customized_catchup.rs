use crate::dijkstra::elimination_tree::parallelization::{SeparatorBasedParallelCustomization, SeparatorBasedPerfectParallelCustomization};
use rayon::prelude::*;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::floating_time_dependent::{
    FlWeight, MergeBuffers, PLFTarget, PartialShortcutGraph, Shortcut, TDGraph, TTFPoint, Timestamp,
};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, LinkIterable, NodeId, NodeIdT, Reversed};
use rust_road_router::report::{push_context, report_time};
use rust_road_router::util::in_range_option::InRangeOption;
use scoped_tls::scoped_thread_local;
use std::cell::RefCell;
use std::cmp::{min, Ordering};
use std::ops::Range;

// Reusable buffers for main CATCHUp customization, to reduce allocations
scoped_thread_local!(static MERGE_BUFFERS: RefCell<MergeBuffers>);
// Workspaces for static CATCHUp precustomization - similar to regular static customization - see parent module
scoped_thread_local!(static UPWARD_WORKSPACE: RefCell<Vec<(FlWeight, FlWeight)>>);
scoped_thread_local!(static DOWNWARD_WORKSPACE: RefCell<Vec<(FlWeight, FlWeight)>>);
// Workspace for perfect static CATCHUp precustomization - here we just need one for both directions
// because we map to the edge id instead of the values.
scoped_thread_local!(static PERFECT_WORKSPACE: RefCell<Vec<InRangeOption<EdgeId>>>);

pub fn customize_internal<'a, 'b: 'a>(cch: &'a CCH, metric: &'b TDGraph) -> (Vec<Vec<TTFPoint>>, Vec<Vec<TTFPoint>>) {
    let m = cch.head.len();

    // these will contain our customized shortcuts
    let mut upward: Vec<_> = std::iter::repeat_with(|| Shortcut::new(None, metric)).take(m).collect();
    let mut downward: Vec<_> = std::iter::repeat_with(|| Shortcut::new(None, metric)).take(m).collect();

    // start with respecting - set shortcuts to respective original edge.
    let subctxt = push_context("weight_applying".to_string());
    report_time("TD-CCH apply weights", || {
        upward
            .par_iter_mut()
            .zip(downward.par_iter_mut())
            .zip(cch.cch_edge_to_orig_arc.par_iter())
            .for_each(|((up_weight, down_weight), (up_arcs, down_arcs))| {
                assert!(up_arcs.len() <= 1);
                assert!(down_arcs.len() <= 1);
                for &EdgeIdT(up_arc) in up_arcs {
                    *up_weight = Shortcut::new(Some(up_arc), metric);
                }
                for &EdgeIdT(down_arc) in down_arcs {
                    *down_weight = Shortcut::new(Some(down_arc), metric);
                }
            });
    });
    drop(subctxt);

    // This is the routine for basic static customization with just the upper and lower bounds.
    // It runs completely analogue the standard customization algorithm.
    let customize = |nodes: Range<usize>, offset: usize, upward_weights: &mut [Shortcut], downward_weights: &mut [Shortcut]| {
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
                        node_incoming_weights[node as usize] = (down.lower_bound, down.upper_bound);
                        node_outgoing_weights[node as usize] = (up.lower_bound, up.upper_bound);
                    }

                    for (NodeIdT(low_node), Reversed(EdgeIdT(first_edge_id))) in cch.inverted.link_iter(current_node) {
                        let first_down_weight: &Shortcut = &downward_weights[first_edge_id as usize - offset];
                        let first_up_weight: &Shortcut = &upward_weights[first_edge_id as usize - offset];
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
                            relax.0 = std::cmp::min(relax.0, upward_weight.lower_bound + first_down_weight.lower_bound);
                            relax.1 = std::cmp::min(relax.1, upward_weight.upper_bound + first_down_weight.upper_bound);
                            let relax = unsafe { node_incoming_weights.get_unchecked_mut(node as usize) };
                            relax.0 = std::cmp::min(relax.0, downward_weight.lower_bound + first_up_weight.lower_bound);
                            relax.1 = std::cmp::min(relax.1, downward_weight.upper_bound + first_up_weight.upper_bound);
                        }
                    }

                    for (((node, down), up), _edge_id) in cch
                        .neighbor_iter(current_node)
                        .zip(&mut downward_weights[edges.clone()])
                        .zip(&mut upward_weights[edges.clone()])
                        .zip(edges)
                    {
                        down.lower_bound = node_incoming_weights[node as usize].0;
                        down.upper_bound = node_incoming_weights[node as usize].1;
                        up.lower_bound = node_outgoing_weights[node as usize].0;
                        up.upper_bound = node_outgoing_weights[node as usize].1;
                        down.update_is_constant();
                        up.update_is_constant();
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
    let customize_perfect = |nodes: Range<usize>, upward: *mut Shortcut, downward: *mut Shortcut| {
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
                                (*upward.add(other_edge_id as usize)).upper_bound = min(
                                    (*upward.add(other_edge_id as usize)).upper_bound,
                                    (*upward.add(edge_id as usize)).upper_bound + (*upward.add(shortcut_edge_id as usize)).upper_bound,
                                );
                                (*upward.add(other_edge_id as usize)).lower_bound = min(
                                    (*upward.add(other_edge_id as usize)).lower_bound,
                                    (*upward.add(edge_id as usize)).lower_bound + (*upward.add(shortcut_edge_id as usize)).lower_bound,
                                );

                                (*upward.add(edge_id as usize)).upper_bound = min(
                                    (*upward.add(edge_id as usize)).upper_bound,
                                    (*upward.add(other_edge_id as usize)).upper_bound + (*downward.add(shortcut_edge_id as usize)).upper_bound,
                                );
                                (*upward.add(edge_id as usize)).lower_bound = min(
                                    (*upward.add(edge_id as usize)).lower_bound,
                                    (*upward.add(other_edge_id as usize)).lower_bound + (*downward.add(shortcut_edge_id as usize)).lower_bound,
                                );

                                (*downward.add(other_edge_id as usize)).upper_bound = min(
                                    (*downward.add(other_edge_id as usize)).upper_bound,
                                    (*downward.add(edge_id as usize)).upper_bound + (*downward.add(shortcut_edge_id as usize)).upper_bound,
                                );
                                (*downward.add(other_edge_id as usize)).lower_bound = min(
                                    (*downward.add(other_edge_id as usize)).lower_bound,
                                    (*downward.add(edge_id as usize)).lower_bound + (*downward.add(shortcut_edge_id as usize)).lower_bound,
                                );

                                (*downward.add(edge_id as usize)).upper_bound = min(
                                    (*downward.add(edge_id as usize)).upper_bound,
                                    (*downward.add(other_edge_id as usize)).upper_bound + (*upward.add(shortcut_edge_id as usize)).upper_bound,
                                );
                                (*downward.add(edge_id as usize)).lower_bound = min(
                                    (*downward.add(edge_id as usize)).lower_bound,
                                    (*downward.add(other_edge_id as usize)).lower_bound + (*upward.add(shortcut_edge_id as usize)).lower_bound,
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

    // parallelize precusotmization
    let static_customization = SeparatorBasedParallelCustomization::new(cch, customize, customize);
    let static_perfect_customization = SeparatorBasedPerfectParallelCustomization::new(cch, customize_perfect, customize_perfect);

    // routine to disable shortcuts for which the perfect precustomization determined them to be irrelevant
    let disable_dominated = |(shortcut, &lower_bound): (&mut Shortcut, &FlWeight)| {
        // shortcut contains shortest path length, lower bound the length of the specific path represented by the shortcut (not necessarily the shortest)
        if shortcut.upper_bound.fuzzy_lt(lower_bound) {
            shortcut.required = false;
            shortcut.lower_bound = FlWeight::INFINITY;
            shortcut.upper_bound = FlWeight::INFINITY;
        } else {
            // reset shortcut lower bound from path to actual shortcut bound
            shortcut.lower_bound = lower_bound;
        }
    };

    // block for main CATCHUp customization
    {
        let subctxt = push_context("main".to_string());

        // use separator based parallelization
        let customization = SeparatorBasedParallelCustomization::new(
            cch,
            // routines created in this function
            // we customize many cells in parallel - so iterate over triangles sequentially
            create_customization_fn(&cch, metric, SeqIter(&cch)),
            // the final separator can only be customized, once everything else is done, but it still takes up a significant amount of time
            // But we can still parallelize the processing of edges from one node within this separator.
            create_customization_fn(&cch, metric, ParIter(&cch)),
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

    println!("Finished main part, now collecting TTFs");

    let shortcut_graph = PartialShortcutGraph::new(metric, &upward, &downward, 0);
    let mut merge_buffers = MergeBuffers::new();
    let mut target = merge_buffers.unpacking_target.push_plf();
    let mut tmp = merge_buffers.unpacking_tmp;

    let upward_weights = upward
        .iter()
        .map(|s| {
            while let Some(_) = target.pop() {}
            s.reconstruct_exact_ttf(Timestamp::ZERO, Timestamp::new(86400.0), &shortcut_graph, &mut target, &mut tmp);
            target.to_vec()
        })
        .collect::<Vec<Vec<TTFPoint>>>();

    let downward_weights = downward
        .iter()
        .map(|s| {
            while let Some(_) = target.pop() {}
            s.reconstruct_exact_ttf(Timestamp::ZERO, Timestamp::new(86400.0), &shortcut_graph, &mut target, &mut tmp);
            target.to_vec()
        })
        .collect::<Vec<Vec<TTFPoint>>>();

    (upward_weights, downward_weights)
}

// Encapsulates the creation of the CATCHUp main customization lambdas
// The function signature gives us some additional control of lifetimes and stuff
fn create_customization_fn<'s, F: 's>(cch: &'s CCH, metric: &'s TDGraph, merge_iter: F) -> impl Fn(Range<usize>, usize, &mut [Shortcut], &mut [Shortcut]) + 's
where
    for<'p> F: ForEachIter<'p, 's, Shortcut>,
{
    move |nodes, edge_offset, upward: &mut [Shortcut], downward: &mut [Shortcut]| {
        // for all nodes we should currently process
        for current_node in nodes {
            let (upward_below, upward_above) = upward.split_at_mut(cch.first_out[current_node as usize] as usize - edge_offset);
            let upward_active = &mut upward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let (downward_below, downward_above) = downward.split_at_mut(cch.first_out[current_node as usize] as usize - edge_offset);
            let downward_active = &mut downward_above[0..cch.neighbor_edge_indices(current_node as NodeId).len()];
            let shortcut_graph = PartialShortcutGraph::new(metric, upward_below, downward_below, edge_offset);

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
                                Ordering::Less => current_iter.next(),
                                Ordering::Greater => other_iter.next(),
                                Ordering::Equal => {
                                    // lower triangle
                                    triangles.push((*edge_from_cur_id, *edge_from_oth_id));

                                    current_iter.next();
                                    other_iter.next()
                                }
                            };
                        }
                        for &edges in &triangles {
                            // main work happening here
                            upward_shortcut.merge(edges, &shortcut_graph, &mut buffers);
                        }
                        upward_shortcut.finalize_bounds(&shortcut_graph);
                        for &(up, down) in &triangles {
                            // an here
                            downward_shortcut.merge((down, up), &shortcut_graph, &mut buffers);
                        }
                        downward_shortcut.finalize_bounds(&shortcut_graph);
                    });
                },
            );
        }
    }
}

trait ForEachIter<'s, 'c, S> {
    fn for_each(
        &self,
        current_node: NodeId,
        upward_active: &'s mut [S],
        downward_active: &'s mut [S],
        f: impl Send + Sync + Fn((((&'c NodeId, EdgeId), &'s mut S), &'s mut S)),
    );
}

struct SeqIter<'c>(&'c CCH);

impl<'s, 'c, S> ForEachIter<'s, 'c, S> for SeqIter<'c> {
    fn for_each(
        &self,
        current_node: NodeId,
        upward_active: &'s mut [S],
        downward_active: &'s mut [S],
        f: impl Send + Sync + Fn((((&'c NodeId, EdgeId), &'s mut S), &'s mut S)),
    ) {
        self.0.head[self.0.neighbor_edge_indices_usize(current_node)]
            .iter()
            .zip(self.0.neighbor_edge_indices(current_node))
            .zip(upward_active.iter_mut())
            .zip(downward_active.iter_mut())
            .for_each(f);
    }
}

struct ParIter<'c>(&'c CCH);

impl<'s, 'c, S: 's> ForEachIter<'s, 'c, S> for ParIter<'c>
where
    S: Send,
    &'s mut [S]: IntoParallelIterator<Item = &'s mut S>,
    <&'s mut [S] as IntoParallelIterator>::Iter: IndexedParallelIterator,
{
    fn for_each(
        &self,
        current_node: NodeId,
        upward_active: &'s mut [S],
        downward_active: &'s mut [S],
        f: impl Send + Sync + Fn((((&'c NodeId, EdgeId), &'s mut S), &'s mut S)),
    ) {
        self.0.head[self.0.neighbor_edge_indices_usize(current_node)]
            .par_iter()
            .zip_eq(self.0.neighbor_edge_indices(current_node))
            .zip_eq(upward_active.par_iter_mut())
            .zip_eq(downward_active.par_iter_mut())
            .for_each(f);
    }
}
