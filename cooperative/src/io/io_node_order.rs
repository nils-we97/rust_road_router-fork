use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::Load;
use std::error::Error;
use std::path::Path;

pub fn load_node_order(directory: &Path) -> Result<NodeOrder, Box<dyn Error>> {
    let order = Vec::load_from(directory.join("order"))?;
    Ok(NodeOrder::from_node_order(order))
}
