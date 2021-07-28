use super::*;

use crate::algo::a_star::*;
use crate::algo::dijkstra::gen_topo_dijkstra::*;
use crate::algo::topocore::*;
use crate::datastr::rank_select_map::*;

pub struct Penalty<P> {
    virtual_topocore: VirtualTopocore,
    // shortest_path_penalized: query::SkipLowDegServer<VirtualTopocoreGraph<OwnedGraph>, DefaultOpsWithLinkPath, PotentialForPermutated<P>, true, true>,
    // shortest_path_penalized: query::BiDirCoreServer<PotentialForPermutated<P>, AlternatingDirs>,
    shortest_path_penalized: query::MultiThreadedBiDirSkipLowDegServer<PotentialForPermutated<P>>,
    alternative_graph_dijkstra: query::SkipLowDegServer<AlternativeGraph<VirtualTopocoreGraph<OwnedGraph>>, DefaultOps, ZeroPotential, true, true>,
    reversed: ReversedGraphWithEdgeIds,
    tail: Vec<NodeId>,
    edge_penelized: BitVec,
    edges_to_reset: Vec<EdgeId>,
}

impl<P: Potential + Send + Clone> Penalty<P> {
    pub fn new<G>(graph: &G, forward_potential: P, backward_potential: P) -> Self
    where
        G: LinkIterable<NodeIdT>,
        OwnedGraph: BuildPermutated<G>,
    {
        let (main_graph, _, virtual_topocore) = VirtualTopocoreGraph::new_topo_dijkstra_graphs(graph);
        let m = main_graph.num_arcs();

        let reversed = ReversedGraphWithEdgeIds::reversed(&main_graph);
        let mut edge_id = 0;
        let mut tail = vec![0; m];
        for node in 0..main_graph.num_nodes() {
            for _ in LinkIterable::<NodeIdT>::link_iter(&main_graph, node as NodeId) {
                tail[edge_id] = node as NodeId;
                edge_id += 1;
            }
        }
        Self {
            // shortest_path_penalized: query::SkipLowDegServer::new(
            // shortest_path_penalized: query::BiDirCoreServer::new(
            shortest_path_penalized: query::MultiThreadedBiDirSkipLowDegServer::new(
                main_graph.clone(),
                PotentialForPermutated {
                    order: virtual_topocore.order.clone(),
                    potential: forward_potential,
                },
                PotentialForPermutated {
                    order: virtual_topocore.order.clone(),
                    potential: backward_potential,
                },
                // DefaultOpsWithLinkPath::default(),
            ),
            alternative_graph_dijkstra: query::SkipLowDegServer::new(
                AlternativeGraph {
                    graph: main_graph,
                    contained_edges: FastClearBitVec::new(m),
                },
                ZeroPotential(),
                DefaultOps::default(),
            ),
            virtual_topocore,
            reversed,
            edge_penelized: BitVec::new(m),
            edges_to_reset: Vec::new(),
            tail,
        }
    }

    pub fn alternatives(&mut self, mut query: Query) -> Option<()> {
        query.permutate(&self.virtual_topocore.order);
        let core_from = self.virtual_topocore.bridge_node(query.from()).unwrap_or(query.from());
        let core_to = self.virtual_topocore.bridge_node(query.to()).unwrap_or(query.to());
        let query = Query { from: core_from, to: core_to };
        self.alternative_graph_dijkstra.graph_mut().clear();

        if let Some(mut result) = report_time_with_key("initial_query", "initial_query", || self.shortest_path_penalized.query(query)).found() {
            let base_dist = result.distance();
            report!("base_dist", base_dist);

            let max_orig_dist = base_dist * 25 / 20;
            let rejoin_penalty = base_dist / 500;

            let max_penalized_dist = max_orig_dist * 25 / 20 + 2 * rejoin_penalty;

            let mut path = result.node_path();
            let mut path_edges = result.edge_path();
            let shortest_path_penalized = &mut self.shortest_path_penalized;
            let alternative_graph_dijkstra = &mut self.alternative_graph_dijkstra;

            alternative_graph_dijkstra.graph_mut().add_edges(&path_edges);
            let path_orig_len: Weight = path_edges
                .iter()
                .map(|&EdgeIdT(e)| alternative_graph_dijkstra.graph().graph.graph.weight()[e as usize])
                .sum();
            debug_assert_eq!(base_dist, path_orig_len);

            let mut iterations_ctxt = push_collection_context("iterations".to_string());

            let mut i = 0;
            let mut s = 0;
            loop {
                i += 1;
                let _iteration_ctxt = iterations_ctxt.push_collection_item();
                report!("iteration", i);
                for &EdgeIdT(edge) in &path_edges {
                    let weight = shortest_path_penalized.graph().graph.weight()[edge as usize];
                    shortest_path_penalized.set_edge_weight(edge, weight * 11 / 10);
                    if !self.edge_penelized.get(edge as usize) {
                        self.edge_penelized.set(edge as usize);
                        self.edges_to_reset.push(edge);
                    }
                }
                let dists: Vec<_> = std::iter::once(0)
                    .chain(path_edges.iter().scan(0, |state, &EdgeIdT(edge)| {
                        *state += shortest_path_penalized.graph().graph.weight()[edge as usize];
                        Some(*state)
                    }))
                    .collect();
                let total_penalized_dist = dists.last().unwrap();
                for (&[tail, head], &[tail_dist, head_dist]) in path.array_windows::<2>().zip(dists.array_windows::<2>()) {
                    for (NodeIdT(rev_head), Reversed(EdgeIdT(edge))) in self.reversed.link_iter(head) {
                        if rev_head != tail {
                            let weight = shortest_path_penalized.graph().graph.weight()[edge as usize];
                            shortest_path_penalized.set_edge_weight(edge, weight + rejoin_penalty * head_dist / total_penalized_dist);
                            if !self.edge_penelized.get(edge as usize) {
                                self.edge_penelized.set(edge as usize);
                                self.edges_to_reset.push(edge);
                            }
                        }
                    }

                    for (NodeIdT(edge_head), EdgeIdT(edge)) in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&alternative_graph_dijkstra.graph().graph, tail) {
                        if edge_head != head {
                            let weight = shortest_path_penalized.graph().graph.weight()[edge as usize];
                            shortest_path_penalized.set_edge_weight(edge, weight + rejoin_penalty * tail_dist / total_penalized_dist);
                            if !self.edge_penelized.get(edge as usize) {
                                self.edge_penelized.set(edge as usize);
                                self.edges_to_reset.push(edge);
                            }
                        }
                    }
                }

                let (result, time) = measure(|| shortest_path_penalized.distance_with_cap(query, max_penalized_dist, max_orig_dist));
                // let (result, time) = measure(|| shortest_path_penalized.distance_with_cap(query, max_penalized_dist));
                report!("running_time_ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
                let mut result = if let Some(result) = result.found() {
                    result
                } else {
                    dbg!("search pruned to death");
                    break;
                };
                let penalty_dist = result.distance();
                report!("penalty_dist", penalty_dist);

                path = result.node_path();
                path_edges = result.edge_path();
                let path_orig_len: Weight = path_edges
                    .iter()
                    .map(|&EdgeIdT(e)| alternative_graph_dijkstra.graph().graph.graph.weight()[e as usize])
                    .sum();
                report!("orig_dist", path_orig_len);
                debug_assert!(path_orig_len >= base_dist);

                let mut feasable = false;
                let new_parts: Vec<_> = path_edges
                    .split(|&EdgeIdT(edge)| alternative_graph_dijkstra.graph().contained_edges.get(edge as usize))
                    .filter(|part| !part.is_empty())
                    .collect();

                for new_part in new_parts {
                    let part_start = self.tail[new_part[0].0 as usize];
                    let part_end = alternative_graph_dijkstra.graph().graph.graph.head()[new_part.last().unwrap().0 as usize];

                    let part_dist: Weight = new_part
                        .iter()
                        .map(|&EdgeIdT(e)| alternative_graph_dijkstra.graph().graph.graph.weight()[e as usize])
                        .sum();

                    if part_dist * 10 > base_dist {
                        let _blocked = block_reporting();
                        if part_dist * 20
                            <= alternative_graph_dijkstra
                                .query(Query {
                                    from: part_start,
                                    to: part_end,
                                })
                                .distance()
                                .map(|d| d * 25)
                                .unwrap_or(INFINITY)
                        {
                            feasable = true;
                            break;
                        }
                    }
                }

                report!("feasable", feasable);
                if feasable {
                    alternative_graph_dijkstra.graph_mut().add_edges(&path_edges);
                    s += 1;
                }

                if path_orig_len > max_orig_dist || penalty_dist > max_penalized_dist || s >= 10 {
                    dbg!(path_orig_len > max_orig_dist);
                    dbg!(penalty_dist > max_penalized_dist);
                    dbg!(s > 10);
                    break;
                }
            }

            drop(iterations_ctxt);

            report!("num_iterations", i);

            // XBDV

            for edge in self.edges_to_reset.drain(..) {
                let e = edge as usize;
                self.edge_penelized.unset_all_around(e as usize);
                shortest_path_penalized.set_edge_weight(edge, alternative_graph_dijkstra.graph().graph.graph.weight()[e as usize]);
            }

            Some(())
        } else {
            None
        }
    }

    pub fn potentials(&self) -> impl Iterator<Item = &PotentialForPermutated<P>> {
        // std::iter::once(self.shortest_path_penalized.potential())
        self.shortest_path_penalized.potentials()
    }
}

struct AlternativeGraph<G> {
    graph: G,
    contained_edges: FastClearBitVec,
}

impl<G> AlternativeGraph<G> {
    fn add_edges(&mut self, edges: &[EdgeIdT]) {
        for &EdgeIdT(edge) in edges {
            self.contained_edges.set(edge as usize);
        }
    }

    fn clear(&mut self) {
        self.contained_edges.clear()
    }
}

impl<G: Graph> Graph for AlternativeGraph<G> {
    fn degree(&self, node: NodeId) -> usize {
        self.graph.degree(node)
    }
    fn num_nodes(&self) -> usize {
        self.graph.num_nodes()
    }
    fn num_arcs(&self) -> usize {
        self.graph.num_arcs()
    }
}

impl<G: LinkIterable<L> + EdgeIdGraph, L> LinkIterable<L> for AlternativeGraph<G> {
    type Iter<'a> = FilteredLinkIter<'a, <G as LinkIterable<L>>::Iter<'a>>;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        FilteredLinkIter {
            iter: self.graph.link_iter(node),
            edge_ids: self.graph.neighbor_edge_indices_usize(node),
            contained_edges: &self.contained_edges,
        }
    }
}

impl<G: SymmetricDegreeGraph> SymmetricDegreeGraph for AlternativeGraph<G> {
    fn symmetric_degree(&self, node: NodeId) -> SymmetricDeg {
        self.graph.symmetric_degree(node)
    }
}

struct FilteredLinkIter<'a, I> {
    iter: I,
    edge_ids: std::ops::Range<usize>,
    contained_edges: &'a FastClearBitVec,
}

impl<'a, L, I: Iterator<Item = L>> Iterator for FilteredLinkIter<'a, I> {
    type Item = L;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(e) = self.edge_ids.next() {
            let next = self.iter.next();
            if self.contained_edges.get(e) {
                return next;
            }
        }
        None
    }
}
