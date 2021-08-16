use std::path::Path;
use rust_road_router::datastr::node_order::NodeOrder;
use std::error::Error;
use rust_road_router::io::Load;

pub fn load_node_order(directory: &Path) -> Result<NodeOrder, Box<dyn Error>> {
    let order = Vec::load_from(directory.join("order"))?;
    Ok(NodeOrder::from_node_order(order))
}