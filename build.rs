use std::env;

fn main() {
    let current_dir = env::current_dir().unwrap();
    let indicator_path = current_dir.join("indicator.pio");

    println!("cargo:rerun-if-changed={}", indicator_path.display());
}