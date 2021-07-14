#![feature(generic_associated_types)]

pub mod io;
pub mod graph;
pub mod dijkstra;
pub mod visualization;
pub mod experiments;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}