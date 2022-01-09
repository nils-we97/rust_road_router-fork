use std::error::Error;
use std::path::Path;

use kdtree::kdtree::{Kdtree, KdtreePointTrait};

use rust_road_router::io::Load;
use rust_road_router::report::measure;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PopulationGridEntry {
    pub id: usize,
    pub coords: [f64; 2],
}

impl PopulationGridEntry {
    pub fn new(id: usize, lon: f64, lat: f64) -> Self {
        Self { id, coords: [lon, lat] }
    }

    pub fn from_coords(lon: f32, lat: f32) -> Self {
        Self {
            id: 0,
            coords: [lon as f64, lat as f64],
        }
    }
}

/// loads the entire population grid
pub fn load_population_grid(directory: &Path) -> Result<(Kdtree<PopulationGridEntry>, Vec<u32>), Box<dyn Error>> {
    // fetch preprocessed population density and coordinates
    let ((longitude, latitude, population), time) = measure(|| {
        let longitude: Vec<f64> = Vec::load_from(directory.join("grid_x")).unwrap();
        let latitude: Vec<f64> = Vec::load_from(directory.join("grid_y")).unwrap();
        let population: Vec<u32> = Vec::load_from(directory.join("population")).unwrap();

        (longitude, latitude, population)
    });
    println!("Loaded population data in {} ms", time.as_secs_f64() * 1000.0);

    // transform into single data structure
    let mut entries = longitude
        .iter()
        .zip(latitude.iter())
        .enumerate()
        .map(|(id, (&lon, &lat))| PopulationGridEntry::new(id, lon, lat))
        .collect::<Vec<PopulationGridEntry>>();

    Ok((Kdtree::new(&mut entries), population))
}

impl KdtreePointTrait for PopulationGridEntry {
    #[inline] // the inline on this method is important! Without it there is ~25% speed loss on the tree when cross-crate usage.
    fn dims(&self) -> &[f64] {
        &self.coords
    }
}
