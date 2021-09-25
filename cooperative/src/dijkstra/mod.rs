use rust_road_router::datastr::graph::{EdgeIdT, LinkIterable, NodeId, NodeIdT};

pub mod capacity_dijkstra_ops;
pub mod model;
pub mod potentials;
pub mod server;

// convert graph to LinkIterable in order to avoid compiler errors
fn get_neighbors<G: LinkIterable<(NodeIdT, EdgeIdT)>>(graph: &G, node: NodeId) -> G::Iter<'_> {
    graph.link_iter(node)
}
