// CATCHUp query experiments with pregenerated queries with source and target drawn uniformly at random.
// Takes as input one directory arg which should contain the all data.

use std::{env, error::Error, path::Path};

use rust_road_router::{
    algo::customizable_contraction_hierarchy::*,
    cli::CliErr,
    datastr::graph::floating_time_dependent::{shortcut_graph::CustomizedGraphReconstrctor, *},
    io::*,
};

fn main() -> Result<(), Box<dyn Error>> {
    let arg = &env::args().skip(1).next().ok_or(CliErr("No directory arg given"))?;
    let path = Path::new(arg);

    let graph = TDGraph::reconstruct_from(&path)?;

    let cch_folder = path.join("cch");
    let cch = CCHReconstrctor(&graph).reconstruct_from(&cch_folder)?;

    let customized_folder = path.join("customized");

    let td_cch_graph = CustomizedGraphReconstrctor {
        original_graph: &graph,
        first_out: cch.first_out(),
        head: cch.head(),
    }
    .reconstruct_from(&customized_folder)?;

    let (unique_down, unique_up) = td_cch_graph.unique_path_edges();
    eprintln!("down: {}/{}", unique_down.count_ones(), unique_down.len());
    eprintln!("up: {}/{}", unique_up.count_ones(), unique_up.len());

    Ok(())
}
