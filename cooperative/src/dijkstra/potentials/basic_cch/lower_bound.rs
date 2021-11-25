use crate::graph::capacity_graph::CapacityGraph;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::report::measure;

/// init lowerbound A*-potential from CCH
pub fn init_cch_potential(graph: &CapacityGraph, order: NodeOrder) -> CCHPotData {
    let (cch, time) = measure(|| CCH::fix_order_and_build(graph, order));
    println!("CCH created in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let (cch_pot_data, time) = measure(|| CCHPotData::new(&cch, graph));
    println!("CCH customized in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    cch_pot_data
}
