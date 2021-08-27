use std::env;
use std::error::Error;

use rust_road_router::cli::CliErr;
use std::path::Path;
//use cooperative::experiments::run_td_server::run_td_server;
//use cooperative::experiments::run_server::run_server;

fn main() -> Result<(), Box<dyn Error>> {
    //let arg = &env::args().skip(1).next().ok_or(CliErr("No directory arg given"))?;
    //let path = Path::new(arg);

    /*let population_directory = env::current_dir()?
    .parent().unwrap()
    .join("graphs")
    .join("population_grid_germany");*/

    //run_server_with_potentials(path);
    Ok(())
}
