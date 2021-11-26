use rust_road_router::algo::customizable_contraction_hierarchy::ftd_cch::customize_internal;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::floating_time_dependent::{PartialShortcutGraph, Shortcut, TDGraph, TTFPoint};

pub fn customize_ptv_graph(cch: &CCH, graph: &TDGraph) -> (Vec<Vec<TTFPoint>>, Vec<Vec<TTFPoint>>) {
    // retrieve shortcuts
    let (upward_shortcuts, downward_shortcuts) = customize_internal(cch, graph);

    let shortcut_graph = PartialShortcutGraph::new(graph, &upward_shortcuts, &downward_shortcuts, 0);

    let upward_weights = extract_shortcuts(&upward_shortcuts, &shortcut_graph);
    let downward_weights = extract_shortcuts(&downward_shortcuts, &shortcut_graph);

    (upward_weights, downward_weights)
}

fn extract_shortcuts(shortcuts: &Vec<Shortcut>, shortcut_graph: &PartialShortcutGraph) -> Vec<Vec<TTFPoint>> {
    shortcuts
        .iter()
        .map(|shortcut| match shortcut.periodic_ttf(shortcut_graph) {
            Some(x) => x.bound_plfs().0.to_vec(),
            None => vec![],
        })
        .collect::<Vec<Vec<TTFPoint>>>()
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
