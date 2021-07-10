#[cfg(feature = "chpot-alt")]
use crate::algo::alt::ALTPotential;
#[cfg(feature = "chpot-cch")]
use crate::{algo::customizable_contraction_hierarchy::*, datastr::node_order::NodeOrder};
use crate::{
    algo::{
        ch_potentials::*,
        dijkstra::{generic_dijkstra::DefaultOps, query::dijkstra::Server as DijkServer},
    },
    datastr::graph::*,
    experiments,
    io::*,
    report::*,
};
use std::{error::Error, path::Path};

use rand::prelude::*;

pub fn run(
    path: &Path,
    modify_travel_time: impl FnOnce(&FirstOutGraph<&[EdgeId], &[NodeId], &[Weight]>, &mut StdRng, &mut [Weight]) -> Result<(), Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    let mut rng = super::rng(Default::default());

    let graph = WeightedGraphReconstructor("travel_time").reconstruct_from(&path)?;
    let mut modified_travel_time = graph.weight().to_vec();

    modify_travel_time(&graph.borrowed(), &mut rng, &mut modified_travel_time)?;
    let modified_graph = FirstOutGraph::new(graph.first_out(), graph.head(), &modified_travel_time[..]);

    let mut algo_runs_ctxt = push_collection_context("algo_runs".to_string());

    let core_ids = core_affinity::get_core_ids().unwrap();
    core_affinity::set_for_current(core_ids[0]);

    #[cfg(feature = "chpot-cch")]
    let cch = {
        let _blocked = block_reporting();
        let order = NodeOrder::from_node_order(Vec::load_from(path.join("cch_perm"))?);
        CCH::fix_order_and_build(&graph, order)
    };

    let potential = {
        #[cfg(feature = "chpot-only-topo")]
        {
            ZeroPotential()
        }
        #[cfg(not(feature = "chpot-only-topo"))]
        {
            #[cfg(feature = "chpot-cch")]
            {
                let _potential_ctxt = algo_runs_ctxt.push_collection_item();
                CCHPotential::new(&cch, &graph)
            }
            #[cfg(feature = "chpot-alt")]
            {
                let _potential_ctxt = algo_runs_ctxt.push_collection_item();
                ALTPotential::new_with_avoid(&graph, 16, &mut rng)
            }
            #[cfg(all(not(feature = "chpot-cch"), not(feature = "chpot-alt")))]
            {
                CHPotential::reconstruct_from(&path.join("lower_bound_ch"))?
            }
        }
    };
    let potential = {
        #[cfg(feature = "chpot-oracle")]
        {
            RecyclingPotential::new(potential)
        }
        #[cfg(not(feature = "chpot-oracle"))]
        {
            potential
        }
    };

    let mut a_star = DijkServer::<_, DefaultOps, _>::with_potential(modified_graph, potential);

    experiments::run_random_queries_with_pre_callback(
        graph.num_nodes(),
        &mut a_star,
        &mut rng,
        &mut algo_runs_ctxt,
        super::chpot::num_queries(),
        |_from, _to, _server| {
            #[cfg(feature = "chpot-oracle")]
            {
                _server.query(Query { from: _from, to: _to });
            }
        },
    );

    Ok(())
}
