use std::env;
use std::error::Error;

use cooperative::experiments::run_td_server::run_td_server;
//use cooperative::experiments::run_server::run_server;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("mecklenburg");

    run_td_server(&graph_directory);

    Ok(())
}