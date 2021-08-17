use std::env;
use std::error::Error;

use cooperative::experiments::run_server_with_potentials::run_server_with_potentials;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?.parent().unwrap().join("graphs").join("baden_wuerttemberg");

    /*let population_directory = env::current_dir()?
    .parent().unwrap()
    .join("graphs")
    .join("population_grid_germany");*/

    run_server_with_potentials(&graph_directory);
    Ok(())
}
