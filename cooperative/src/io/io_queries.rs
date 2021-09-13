use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::NodeId;
use rust_road_router::io::{Load, Store};
use std::error::Error;
use std::path::Path;

/// load queries from a given directory
pub fn load_queries(directory: &Path) -> Result<Vec<TDQuery<Timestamp>>, Box<dyn Error>> {
    let sources = Vec::load_from(directory.join("source"))?;
    let targets = Vec::load_from(directory.join("target"))?;
    let departures = Vec::load_from(directory.join("departure"))?;

    assert!(sources.len() == targets.len() && sources.len() == departures.len());

    let queries = sources
        .iter()
        .zip(targets.iter())
        .zip(departures.iter())
        .map(|((&source, &target), &departure)| TDQuery::new(source, target, departure))
        .collect::<Vec<TDQuery<Timestamp>>>();

    Ok(queries)
}

/// store queries in a given directory
pub fn store_queries(queries: &Vec<TDQuery<Timestamp>>, directory: &Path) -> Result<(), Box<dyn Error>> {
    let sources = queries.iter().map(|q| q.from).collect::<Vec<NodeId>>();
    let targets = queries.iter().map(|q| q.to).collect::<Vec<NodeId>>();
    let departures = queries.iter().map(|q| q.departure).collect::<Vec<Timestamp>>();

    sources.write_to(&directory.join("source"))?;
    targets.write_to(&directory.join("target"))?;
    departures.write_to(&directory.join("departure"))?;

    Ok(())
}
