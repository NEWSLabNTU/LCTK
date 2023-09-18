#![cfg(feature = "with-nalgebra")]

use anyhow::Result;
use wayside_params::infra_v2;

const INFRA_V2_FILE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/infra_v2_example/main.json5",
);

#[test]
fn load_infra_v2() -> Result<()> {
    let params = infra_v2::InfraV2::open(INFRA_V2_FILE)?;
    let _map = params.to_coord_transform_map();

    Ok(())
}
