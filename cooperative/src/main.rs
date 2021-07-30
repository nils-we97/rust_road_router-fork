use std::env;
use std::error::Error;

//use cooperative::experiments::run_td_server::run_td_server;
//use cooperative::experiments::run_server::run_server;
use cooperative::io::extract_scc::extract_largest_scc;

fn main() -> Result<(), Box<dyn Error>> {
    let graph_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("mecklenburg");

    let out_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("mecklenburg-2");


    /*let population_directory = env::current_dir()?
        .parent().unwrap()
        .join("graphs")
        .join("population_grid_germany");*/

    //run_server(&graph_directory, &population_directory);
    extract_largest_scc(&graph_directory, &out_directory)?;

    println!("This should not be printed out, my app crashed before!");

    Ok(())
}