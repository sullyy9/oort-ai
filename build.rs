use std::path::Path;

extern crate rustsourcebundler;
use rustsourcebundler::Bundler;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut bundler = Bundler::new(Path::new("./src/bundle_input.rs"), Path::new("./out.rs"));
    bundler.crate_name("oort-ai");
    bundler.run();
    Ok(())
}
