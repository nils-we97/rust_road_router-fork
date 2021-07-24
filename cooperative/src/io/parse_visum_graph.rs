use std::error::Error;
use std::path::Path;

use rust_road_router::io::{Load, Store};
use std::str::FromStr;

pub fn parse_visum_coords(graph_directory: &Path) -> Result<(), Box<dyn Error>> {
    let lon: Vec<i32> = Vec::load_from(graph_directory.join("longitude_visum"))?;
    let lat: Vec<i32> = Vec::load_from(graph_directory.join("latitude_visum"))?;

    println!("{}", lat[0]);

    // example: 3513149 -> 35.13149
    // use conversion via strings for higher precision
    let lon = convert_coord(&lon);
    let lat = convert_coord(&lat);

    println!("{}", lat[0]);

    lon.write_to(&graph_directory.join("longitude"))?;
    lat.write_to(&graph_directory.join("latitude"))?;

    println!("Successfully converted i32 lat_lng to f32 values!");

    Ok(())
}

pub fn convert_coord(coords: &Vec<i32>) -> Vec<f32> {
    coords
        .iter()
        .map(|&c| {
            let coord_len = c.to_string().len();
            assert!(coord_len == 7 || coord_len == 8, "Invalid length: {}", coord_len);

            let float_str = format!("{}.{}", c / 1000000, c % 1000000);
            f32::from_str(&float_str).unwrap()
        })
        .collect::<Vec<f32>>()
}