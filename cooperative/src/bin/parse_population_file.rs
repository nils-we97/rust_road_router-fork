/*use cooperative::util::cli_args::parse_arg_required;
use proj::Proj;
use rust_road_router::io::Store;
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;*/

use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // parse population directory and population file type
    /*let mut args = std::env::args().skip(1);
    let population_directory: String = parse_arg_required(&mut args, "Population File Directory")?;
    let population_type: String = parse_arg_required(&mut args, "Population File Type [GER/EUR]")?;
    let path = Path::new(&population_directory);

    let records = match population_type.to_uppercase().as_str() {
        "GER" => {
            let population_file_path = path.join("Zensus_Bevoelkerung_100m-Gitter.csv");
            let file = File::open(&population_file_path)?;
            let mut reader = BufReader::new(&file);

            reader.lines().filter_map(|line| {
                if let Ok(line) = line {
                    if !line.starts_with("Gitter_ID") {
                        let fields = line.split(";").collect::<Vec<&str>>();
                        if fields.len() == 4 {
                            let center_x = fields[1].parse::<f64>().expect("failed to parse to f64");
                            let center_y = fields[2].parse::<f64>().expect("failed to parse to f64 #2");
                            let population = fields[3].parse::<u32>().expect("failed to parse to u32");

                            if population > 0 {
                                return Some((center_x, center_y, population));
                            }
                        }
                    }
                }
                None
            })
        }
        "EUR" => {
            let population_file_path = path.join("GEOSTAT_grid_POP_1K_2011_V2_0_1.csv");
            let file = File::open(&population_file_path)?;
            let mut reader = BufReader::new(&file);

            reader.lines().filter_map(|line| {
                if let Ok(line) = line {
                    if !line.starts_with("TOT_P") {
                        let fields = line.split(",").collect::<Vec<&str>>();
                        if fields.len() > 2 {
                            let population = fields[0].parse::<u32>().expect("failed to parse to u32");
                            let coord_line = fields[1].replace("1kmN", "").split("E").collect::<Vec<&str>>();

                            if coord_line.len() == 2 && population > 0 {
                                let center_x = coord_line[1].parse::<f64>().expect("failed to parse e coord");
                                let center_y = coord_line[2].parse::<f64>().expect("failed to parse n coord");

                                return Some((center_x * 1000.0 + 500.0, center_y * 1000.0 + 500.0, population));
                            }
                        }
                    }
                }
                None
            })
        }
        _ => panic!("Invalid population file type. Currently supported: GER/EUR"),
    };

    let transformation = Proj::new_known_crs("EPSG:3035", "EPSG:4326", None).unwrap();
    let size = records.count();
    let mut grid_x = Vec::with_capacity(size);
    let mut grid_y = Vec::with_capacity(size);
    let mut population = Vec::with_capacity(size);

    records.for_each(|(x, y, p)| {
        let (x, y) = transformation.convert((x, y)).unwrap();

        grid_x.push(x);
        grid_y.push(y);
        population.push(p);
    });

    grid_x.write_to(&path.join("grid_x"))?;
    grid_y.write_to(&path.join("grid_y"))?;
    population.write_to(&path.join("population"))?;*/

    Ok(())
}
