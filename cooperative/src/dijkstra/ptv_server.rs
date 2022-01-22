use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use crate::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use crate::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use crate::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use crate::dijkstra::potentials::TDPotential;
use rust_road_router::algo::dijkstra::query::td_dijkstra::TDDijkstraOps;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::{TDGraph, Timestamp};
use rust_road_router::datastr::graph::{Arc, EdgeIdT, Graph, LinkIterable, NodeIdT, Weight, INFINITY};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report::measure;
use std::time::{Duration, Instant};

pub struct PTVQueryServer<Customized> {
    graph: TDGraph,
    dijkstra: DijkstraData<Weight, (), Weight>,
    customized: Customized,
    pub sum_potentials: u64,
}

pub struct PTVQueryResult {
    pub distance: Option<Weight>,
    pub time_potential: Duration,
    pub time_query: Duration,
    pub num_relaxed_arcs: u32,
    pub num_queue_pops: u32,
    pub num_queue_pushs: u32,
}

impl<Customized> PTVQueryServer<Customized> {
    pub fn new(graph: TDGraph, customized: Customized) -> Self {
        let n = graph.num_nodes();

        Self {
            graph,
            dijkstra: DijkstraData::new(n),
            customized,
            sum_potentials: 0,
        }
    }

    // borrow/decompose functions
    pub fn decompose(self) -> (TDGraph, Customized) {
        (self.graph, self.customized)
    }
    pub fn borrow_graph(&self) -> &TDGraph {
        &self.graph
    }

    // we only want to measure the query time, hence we can omit the path calculation
    fn query_internal<Pot: TDPotential>(
        graph: &TDGraph,
        dijkstra: &mut DijkstraData<Weight, (), Weight>,
        query: &TDQuery<Timestamp>,
        pot: &mut Pot,
        sum_potentials: &mut u64,
    ) -> PTVQueryResult {
        let from = query.from();
        let init = query.initial_state();
        let to = query.to();

        let mut result = None;
        let mut num_queue_pops = 0;
        let mut num_queue_pushs = 0;
        let mut num_relaxed_arcs = 0;

        // time-dependent potentials are a little bit more complicated
        // for now, a slight modification of the generic dijkstra code should suffice

        // prepro: initialize potential
        let (_, time_potential) = measure(|| pot.init(from, to, query.departure));

        let start = Instant::now();
        let mut ops = TDDijkstraOps::default();

        // 1. reset data
        dijkstra.queue.clear();
        dijkstra.distances.reset();

        // 2. init dijkstra from start node
        dijkstra.queue.push(State { key: init.key(), node: from });
        dijkstra.distances[from as usize] = init;
        dijkstra.predecessors[from as usize].0 = from;

        // 3. run query
        while let Some(State { node, .. }) = dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == to {
                result = Some(dijkstra.distances[to as usize] - dijkstra.distances[from as usize]);
                break;
            }

            for link in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(graph, &dijkstra.predecessors, NodeIdT(node), &dijkstra.distances[node as usize], &link);

                if ops.merge(&mut dijkstra.distances[link.head() as usize], linked) {
                    let next_distance = &dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), next_distance.clone()).map(|p| p + next_distance.key()) {
                        let next = State {
                            node: link.head(),
                            key: next_key,
                        };
                        if dijkstra.queue.contains_index(next.as_index()) {
                            dijkstra.queue.decrease_key(next);
                        } else {
                            num_queue_pushs += 1;
                            dijkstra.queue.push(next);
                        }
                    }
                }
            }
        }

        let time_query = start.elapsed();
        *sum_potentials += pot.potential(from, init).unwrap_or(0) as u64;

        debug_assert!(
            result.unwrap_or(INFINITY) + 1 >= pot.potential(from, init).unwrap_or(INFINITY),
            "{:#?} {:#?}",
            &result,
            &pot.potential(from, init)
        );

        PTVQueryResult {
            distance: result,
            time_potential,
            time_query,
            num_relaxed_arcs,
            num_queue_pops,
            num_queue_pushs,
        }
    }
}

impl PTVQueryServer<CustomizedMultiMetrics> {
    pub fn query(&mut self, query: &TDQuery<Timestamp>) -> PTVQueryResult {
        let mut pot = MultiMetricPotential::prepare(&mut self.customized);
        Self::query_internal(&self.graph, &mut self.dijkstra, query, &mut pot, &mut self.sum_potentials)
    }
}

impl PTVQueryServer<CustomizedCorridorLowerbound> {
    pub fn query(&mut self, query: &TDQuery<Timestamp>) -> PTVQueryResult {
        let mut pot = CorridorLowerboundPotential::prepare_ptv(&mut self.customized);
        Self::query_internal(&self.graph, &mut self.dijkstra, query, &mut pot, &mut self.sum_potentials)
    }
}

impl<Customized: TDPotential> PTVQueryServer<Customized> {
    pub fn query(&mut self, query: &TDQuery<Timestamp>) -> PTVQueryResult {
        Self::query_internal(&self.graph, &mut self.dijkstra, query, &mut self.customized, &mut self.sum_potentials)
    }
}
