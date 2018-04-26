use time;

pub fn measure<Out, F: FnOnce() -> Out>(name: &str, f: F) -> Out {
    let start = time::now();
    let res = f();
    println!("{}: {}ms", name, (time::now() - start).num_milliseconds());
    res
}

#[derive(Debug)]
pub struct Timer {
    start: time::Tm
}

impl Timer {
    pub fn new() -> Timer {
        Timer { start: time::now() }
    }

    pub fn restart(&mut self) {
        self.start = time::now();
    }

    pub fn report_passed_ms(&self) {
        println!("{}ms", (time::now() - self.start).num_milliseconds());
    }

    pub fn get_passed_ms(&self) -> i64 {
        (time::now() - self.start).num_milliseconds()
    }
}
