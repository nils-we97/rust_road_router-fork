use crate::dijkstra::potentials::TDPotential;
use rust_road_router::algo::dijkstra::query::td_dijkstra::TDDijkstraOps;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::{TDGraph, Timestamp};
use rust_road_router::datastr::graph::{Arc, EdgeIdT, Graph, LinkIterable, NodeIdT, Weight, INFINITY};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report::measure;

pub struct PTVQueryServer<Pot> {
    graph: TDGraph,
    dijkstra: DijkstraData<Weight, (), Weight>,
    potential: Pot,
}

pub struct PTVQueryResult {
    pub distance: Option<Weight>,
    pub time_potential: time::Duration,
    pub time_query: time::Duration,
    pub num_relaxed_arcs: u32,
    pub num_queue_pops: u32,
    pub num_queue_pushs: u32,
}

impl<Pot: TDPotential> PTVQueryServer<Pot> {
    pub fn new_with_potential(graph: TDGraph, potential: Pot) -> PTVQueryServer<Pot> {
        let nodes = graph.num_nodes();

        PTVQueryServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential,
        }
    }

    // borrow/decompose functions
    pub fn decompose(self) -> (TDGraph, Pot) {
        (self.graph, self.potential)
    }
    pub fn borrow_graph(&self) -> &TDGraph {
        &self.graph
    }

    // we only want to measure the query time, hence we can omit the path calculation
    pub fn query(&mut self, query: TDQuery<Timestamp>) -> PTVQueryResult {
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
        let (_, time_potential) = measure(|| self.potential.init(from, to, query.departure));
        let pot = &mut self.potential;

        let start = time::now();
        let mut ops = TDDijkstraOps::default();

        // 1. reset data
        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();

        // 2. init dijkstra from start node
        self.dijkstra.queue.push(State { key: init.key(), node: from });
        self.dijkstra.distances[from as usize] = init;
        self.dijkstra.predecessors[from as usize].0 = from;

        // 3. run query
        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == to {
                result = Some(self.dijkstra.distances[to as usize] - self.dijkstra.distances[from as usize]);
                break;
            }

            for link in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(
                    &self.graph,
                    &self.dijkstra.predecessors,
                    NodeIdT(node),
                    &self.dijkstra.distances[node as usize],
                    &link,
                );

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    let next_distance = &self.dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), next_distance.clone()).map(|p| p + next_distance.key()) {
                        let next = State {
                            node: link.head(),
                            key: next_key,
                        };
                        if self.dijkstra.queue.contains_index(next.as_index()) {
                            self.dijkstra.queue.decrease_key(next);
                        } else {
                            num_queue_pushs += 1;
                            self.dijkstra.queue.push(next);
                        }
                    }
                }
            }
        }

        let time_query = time::now() - start;

        /*println!(
            "Query results: {}, potential: {}",
            result.unwrap_or(INFINITY),
            pot.potential(from, init).unwrap()
        );
        println!(
            "Query times: Potential: {}, Query: {}",
            time_potential.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
            time_dijkstra.to_std().unwrap().as_nanos() as f64 / 1_000_000.0
        );*/
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
