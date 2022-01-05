use cooperative::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use cooperative::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::ptv_server::PTVQueryServer;
use cooperative::io::io_ptv_customization::{load_interval_minima, load_multiple_metrics};
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rand::{thread_rng, Rng};
use rust_road_router::algo::a_star::ZeroPotential;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::TDGraph;
use rust_road_router::datastr::graph::FirstOutGraph;
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::{Load, Reconstruct};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::ops::Add;
use std::path::Path;

/// Compare runtimes of different potentials on the same graph with the same queries
/// Uses a given PTV graph and executes a set of pre-defined queries
/// then executes a set of queries (without updates) on several potentials
///
/// Additional parameters: <path_to_graph> <query_path> <corridor_lowerbound_customized_path> <multi_metrics_customized_path>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, customized_cl, customized_mm) = parse_args()?;
    let path = Path::new(&graph_directory);

    let ((graph, lower_bound), time) = measure(|| {
        (
            TDGraph::reconstruct_from(&path).unwrap(),
            Vec::<u32>::load_from(&path.join("lower_bound")).unwrap(),
        )
    });
    println!("Loaded graph in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load pre-generated queries
    let queries = load_queries(&path.join("queries").join(query_directory))?;

    // init cch
    let order = Vec::load_from(path.join("cch_perm"))?;
    let node_order = NodeOrder::from_node_order(order);
    let cch = CCH::fix_order_and_build(&graph, node_order);

    // ----------------------------------------------------------------------------- //
    // Baseline: naive dijkstra
    // on large graphs with ~7-8 seconds for a single query, this might take quite some time!
    // therefore we sample ~1000 queries, which should be enough to get a statistically valid result
    let mut server = PTVQueryServer::new_with_potential(graph, ZeroPotential());

    let dijkstra_queries = if queries.len() > 1000 {
        let mut rng = thread_rng();
        queries.iter().filter(|_| rng.gen_range(0..queries.len() as u64) < 1000).cloned().collect()
    } else {
        queries.clone()
    };

    let pot_name = format!("Naive Dijkstra ({} queries)", dijkstra_queries.len());
    execute_queries(&mut server, &dijkstra_queries, pot_name.as_str());
    let (graph, _) = server.decompose();

    // ----------------------------------------------------------------------------- //
    // 1st potential: CCH lowerbound
    let lower_bound_graph = FirstOutGraph::new(graph.first_out(), graph.head(), &lower_bound[..]);
    let cch_pot_data = CCHPotData::new(&cch, &lower_bound_graph);
    let cch_lowerbound_pot = cch_pot_data.forward_potential();
    let mut server = PTVQueryServer::new_with_potential(graph, cch_lowerbound_pot);

    execute_queries(&mut server, &queries, "CCH Lowerbound Potential");
    let (graph, cch_lowerbound_pot) = server.decompose();
    drop(cch_lowerbound_pot);
    drop(cch_pot_data);

    // ----------------------------------------------------------------------------- //
    // 2nd potential: Multi-Metric Potential
    let (customized_multi_metric, time) = measure(|| load_multiple_metrics(&path.join("customized").join(customized_mm), &cch).unwrap());
    println!("Loaded customized data in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let multi_metric_pot = MultiMetricPotential::new(&customized_multi_metric);
    let mut server = PTVQueryServer::new_with_potential(graph, multi_metric_pot);
    execute_queries(&mut server, &queries, "Multi Metric Pot");
    let (graph, pot) = server.decompose();

    drop(pot);
    drop(customized_multi_metric);

    // ----------------------------------------------------------------------------- //
    // 3rd potential: Corridor-Lowerbound Potential
    let (customized_corridor_lowerbound, time) = measure(|| load_interval_minima(&path.join("customized").join(customized_cl)).unwrap());
    println!("Loaded customized data in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let corridor_lowerbound_pot = CorridorLowerboundPotential::new(&customized_corridor_lowerbound);
    let mut server = PTVQueryServer::new_with_potential(graph, corridor_lowerbound_pot);

    execute_queries(&mut server, &queries, "Corridor Lowerbound Potential");
    Ok(())
}

fn execute_queries<Pot: TDPotential>(server: &mut PTVQueryServer<Pot>, queries: &Vec<TDQuery<u32>>, pot_name: &str) {
    let mut sum_distances = 0u64;
    let mut num_relaxed_arcs = 0u64;
    let mut num_queue_pops = 0u64;

    let mut time_total = time::Duration::zero();
    let mut time_queries = time::Duration::zero();
    let mut time_potentials = time::Duration::zero();

    queries.iter().enumerate().for_each(|(idx, &query)| {
        let (result, time) = measure(|| server.query(query));

        time_total = time_total.add(time);
        time_queries = time_queries.add(result.time_query);
        time_potentials = time_potentials.add(result.time_potential);

        sum_distances += result.distance.unwrap_or(0) as u64;
        num_relaxed_arcs += result.num_relaxed_arcs as u64;
        num_queue_pops += result.num_queue_pops as u64;

        if (idx + 1) % 1000 == 0 {
            println!("Finished {}/{} queries", idx + 1, queries.len());
        }
    });

    println!("-----------------------------");
    println!("Result for {}:", pot_name);
    println!(
        "Total runtime: {} ms (potential init: {}, query + pot: {})",
        time_total.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_potentials.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_queries.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
    );
    println!(
        "Query statistics: {} relaxed arcs (avg: {}), {} queue pops (avg: {})",
        num_relaxed_arcs,
        num_relaxed_arcs / queries.len() as u64,
        num_queue_pops,
        num_queue_pops / queries.len() as u64
    );
    println!("Total distance: {} (avg: {})", sum_distances, sum_distances / queries.len() as u64);
    println!(
        "Potential estimation: {} (avg: {})",
        server.sum_potentials,
        server.sum_potentials / queries.len() as u64
    );
    println!("-----------------------------");
}

fn parse_args() -> Result<(String, String, String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;

    let customized_cl = parse_arg_required(&mut args, "Customized Corridor-Lowerbound")?;
    let customized_mm = parse_arg_required(&mut args, "Customized Multi-Metric")?;

    Ok((graph_directory, query_directory, customized_cl, customized_mm))
}
