use std::borrow::Borrow;

use rust_road_router::algo::dijkstra::{DefaultOpsWithLinkPath, DijkstraData, DijkstraRun, DijkstraOps, State, Label};
use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{EdgeId, EdgeIdT, Graph, Weight, LinkIterable, Arc, NodeIdT, NodeId};
use rust_road_router::report;
use rust_road_router::report::*;

use crate::dijkstra::model::{CapacityQueryResult, PathResult, TDPathResult};
use crate::dijkstra::td_capacity_dijkstra_ops::TDCapacityDijkstraOps;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::td_capacity_graph::TDCapacityGraph;
use crate::graph::ModifiableWeight;
use rust_road_router::algo::a_star::{Potential, ZeroPotential};
use crate::dijkstra::potentials::TDPotential;
use rust_road_router::datastr::index_heap::Indexing;

pub struct CapacityServer<G, Pot = ZeroPotential> {
    graph: G,
    dijkstra: DijkstraData<Weight, EdgeIdT>,
    potential: Pot,
}

pub trait CapacityServerOps<G, P, Pot = ZeroPotential> {
    fn new(graph: G) -> CapacityServer<G>;
    fn new_with_potential(graph: G, potential: Pot) -> CapacityServer<G, Pot>;
    fn query(&mut self, query: impl GenQuery<Weight> + Clone, update: bool) -> Option<CapacityQueryResult<P>>;
    fn update(&mut self, path: &P);
    fn distance(&mut self, query: impl GenQuery<Weight> + Clone) -> Option<Weight>;
    fn path(&self, query: impl GenQuery<Weight> + Clone) -> P;
    fn path_distance(&self, path: &P) -> Weight;
}

impl<Pot> CapacityServerOps<CapacityGraph, PathResult, Pot> for CapacityServer<CapacityGraph, Pot>
    where Pot: Potential
{
    fn new(graph: CapacityGraph) -> CapacityServer<CapacityGraph> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential: ZeroPotential(),
        }
    }

    fn new_with_potential(graph: CapacityGraph, potential: Pot) -> CapacityServer<CapacityGraph, Pot> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential,
        }
    }


    fn query(&mut self, query: impl GenQuery<Weight> + Clone, update: bool) -> Option<CapacityQueryResult<PathResult>> {
        let query_copy = query.clone();

        let distance = self.distance(query);
        //let (distance, time) = measure(|| self.distance(query));
        //println!("Query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        if distance.is_some() {
            let path = self.path(query_copy);
            if update {
                self.update(&path);
                //let (_, time) = measure(|| self.update(&path));
                //println!("Update took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
            }
            return Some(CapacityQueryResult::new(distance.unwrap(), path));
        }

        return None;
    }

    fn update(&mut self, path: &PathResult) {
        self.graph.increase_weights(&path.edge_path);
    }

    fn distance(&mut self, query: impl GenQuery<Weight>) -> Option<Weight> {
        report!("algo", "Dijkstra with Capacities");

        let to = query.to();
        let mut ops = DefaultOpsWithLinkPath::default();
        let mut dijkstra = DijkstraRun::query(self.graph.borrow(), &mut self.dijkstra, &mut ops, query);

        self.potential.init(to);
        let pot = &mut self.potential;

        let mut result = None;
        let mut num_queue_pops = 0;
        while let Some(node) = dijkstra.next_step_with_potential(|node| pot.potential(node)) {
            num_queue_pops += 1;
            if node == to {
                result = Some(*dijkstra.tentative_distance(node));
                break;
            }
        }

        report!("num_queue_pops", num_queue_pops);
        report!("num_queue_pushs", dijkstra.num_queue_pushs());
        report!("num_relaxed_arcs", dijkstra.num_relaxed_arcs());

        result
    }

    fn path(&self, query: impl GenQuery<Weight>) -> PathResult {
        let mut node_path = Vec::new();
        let mut edge_path = Vec::new();
        node_path.push(query.to());

        while *node_path.last().unwrap() != query.from() {
            let (next_node, next_edge) = self.dijkstra.predecessors[*node_path.last().unwrap() as usize];
            node_path.push(next_node);
            edge_path.push(next_edge.0);
        }
        node_path.reverse();
        edge_path.reverse();

        PathResult::new(node_path, edge_path)
    }

    fn path_distance(&self, path: &PathResult) -> Weight {
        path.edge_path.iter().map(|&edge_id| self.graph.weight(edge_id)).sum()
    }
}

impl<Pot> CapacityServerOps<TDCapacityGraph, TDPathResult, Pot> for CapacityServer<TDCapacityGraph, Pot>
    where Pot: TDPotential
{
    fn new(graph: TDCapacityGraph) -> CapacityServer<TDCapacityGraph> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential: ZeroPotential(),
        }
    }

    fn new_with_potential(graph: TDCapacityGraph, potential: Pot) -> CapacityServer<TDCapacityGraph, Pot> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential,
        }
    }

    fn query(&mut self, query: impl GenQuery<Weight> + Clone, update: bool) -> Option<CapacityQueryResult<TDPathResult>> {
        let distance = self.distance(query.clone());
        //let (distance, time) = measure(|| self.distance(query));
        //println!("//Query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        if distance.is_some() {
            let path = self.path(query);
            if update {
                self.update(&path);
                //let (_, time) = measure(|| self.update(&path));
                //println!("//Update took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
            }
            return Some(CapacityQueryResult::new(distance.unwrap(), path));
        }

        return None;
    }

    fn update(&mut self, path: &TDPathResult) {
        let edges_with_timestamps = path
            .edge_path
            .iter()
            .zip(path.departure.iter())
            .map(|(&a, &b)| (a, b))
            .collect::<Vec<(EdgeId, Timestamp)>>();

        self.graph.increase_weights(&edges_with_timestamps);
    }

    fn distance(&mut self, query: impl GenQuery<Weight>) -> Option<Weight> {
        report!("algo", "TD Dijkstra with Capacities");

        let from = query.from();
        let init = query.initial_state();
        let to = query.to();

        let mut ops = TDCapacityDijkstraOps::default();

        let mut result = None;
        let mut num_queue_pops = 0;
        let mut num_queue_pushs = 0;
        let mut num_relaxed_arcs = 0;

        // time-dependent potentials are a little bit more complicated
        // for now, a slight modification of the generic dijkstra code should suffice

        // prepro: initialize potential
        self.potential.init(to, 0);
        let pot = &mut self.potential;

        // 1. reset data
        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();

        // 2. init dijkstra from start node
        self.dijkstra.queue.push(State { key: init.key(), node: from});
        self.dijkstra.distances[from as usize] = init;
        self.dijkstra.predecessors[from as usize].0 = from;

        // 3. run query
        while let Some(State {node, key}) = self.dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == to {
                result = Some(self.dijkstra.distances[to as usize]);
                break;
            }

            for link in get_neighbors(&self.graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(&self.graph, &self.dijkstra.distances[node as usize], &link);

                if ops.merge(&mut self.dijkstra.distances[node as usize], linked) {
                    self.dijkstra.predecessors[link.head() as usize] = (node, ops.predecessor_link(&link));
                    let next_distance = &self.dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), key).map(|p| p + next_distance.key()) {
                        let next = State { node: link.head(), key: next_key };
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

        report!("num_queue_pops", num_queue_pops);
        report!("num_queue_pushs", num_queue_pushs);
        report!("num_relaxed_arcs", num_relaxed_arcs);

        result
    }

    fn path(&self, query: impl GenQuery<Weight>) -> TDPathResult {
        let mut node_path = Vec::new();
        let mut edge_path = Vec::new();
        node_path.push(query.to());

        // determine path nodes/edges by recursively traversing through the predecessors of the target node
        while *node_path.last().unwrap() != query.from() {
            let (next_node, next_edge) = self.dijkstra.predecessors[*node_path.last().unwrap() as usize];
            node_path.push(next_node);
            edge_path.push(next_edge.0);
        }

        // determine timestamps of departures at each vertex
        // TODO: verify distance == (departure[last] - departure[first])
        let mut departure = Vec::with_capacity(node_path.len());
        let mut current_time = query.initial_state();

        for i in 0..node_path.len() - 1 {
            departure.push(current_time);

            // update travel time by traversing the next edge at the current time
            let ttf = self.graph.travel_time_function(edge_path[i]);
            current_time += ttf.eval(current_time)
        }

        departure.push(current_time); // arrival time at target node

        TDPathResult::new(node_path, edge_path, departure)
    }

    fn path_distance(&self, path: &TDPathResult) -> Weight {
        path.edge_path
            .iter()
            .enumerate()
            .map(|(idx, &edge_id)| self.graph.weight(edge_id, path.departure[idx]))
            .sum()
    }
}

// helper function
fn get_neighbors<G: LinkIterable<(NodeIdT, EdgeIdT)>>(graph: &G, node: NodeId) -> G::Iter<'_> {
    graph.link_iter(node)
}