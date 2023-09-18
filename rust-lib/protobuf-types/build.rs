use anyhow::{format_err, Result};
use itertools::Itertools;
use prost_build::Config;
use std::path::Path;

fn main() -> Result<()> {
    // Generate .rs files from protobuf
    let input_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("protobuf");
    let proto_paths: Vec<_> = glob::glob(
        input_dir
            .join("*.proto")
            .to_str()
            .ok_or_else(|| format_err!("the path is not unicode"))?,
    )?
    .try_collect()?;

    // generate protobuf types
    Config::new()
        .type_attribute(".", "#[derive(::serde::Serialize, ::serde::Deserialize)]")
        .compile_protos(&proto_paths, &[input_dir])?;

    Ok(())
}
