use std::env;
use std::error::Error;

use cooperative::experiments::run_td_server::run_td_server;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?.parent().unwrap().join("graphs").join("mecklenburg-2");

    /*let population_directory = env::current_dir()?
    .parent().unwrap()
    .join("graphs")
    .join("population_grid_germany");*/

    run_td_server(&graph_directory);
    Ok(())
}
