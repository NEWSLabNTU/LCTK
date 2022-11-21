use anyhow::{bail, ensure, format_err, Result};
use pcd_format::{LibpclPoint, NewslabV1Point};
use std::{
    f64,
    path::{Path, PathBuf},
};
use structopt::StructOpt;

mod types;
mod utils;
use crate::types::FileFormat;

// use types::FileFormat;

#[derive(Debug, Clone, StructOpt)]
enum Opts {
    Info {
        file: PathBuf,
    },
    /// Point cloud file conversion.
    Convert {
        input_path: PathBuf,
        output_path: PathBuf,
    },
    DeviceTime {
        file: PathBuf,
    },
}

fn main() -> Result<()> {
    let opts = Opts::from_args();

    match opts {
        Opts::Info { file } => {
            info(file)?;
        }
        Opts::Convert {
            input_path,
            output_path,
        } => {
            convert(input_path, output_path)?;
        }
        Opts::DeviceTime { file } => {
            crate::utils::time(file)?;
        }
    }

    Ok(())
}

fn info(file: impl AsRef<Path>) -> Result<()> {
    let file = file.as_ref();

    ensure!(
        file.extension().map(|ext| ext == "pcd").unwrap_or(false),
        "file name must ends with '.pcd', but get '{}'",
        file.display()
    );

    let reader = pcd_rs::DynReader::open(file)?;
    let fields = &reader.meta().field_defs;

    println!("name\ttype\tcount");
    fields.iter().for_each(|field| {
        let pcd_rs::FieldDef {
            ref name,
            kind,
            count,
        } = *field;

        println!("{}\t{:?}\t{}", name, kind, count);
    });

    Ok(())
}

fn convert(input_path: PathBuf, output_path: PathBuf) -> Result<()> {
    let input_format = guess_file_format(&input_path).ok_or_else(|| {
        format_err!(
            "cannot guess format of input file '{}'",
            input_path.display()
        )
    })?;

    let output_format = guess_file_format(&output_path);

    match (input_format, output_format) {
        (FileFormat::LibpclPcd, Some(FileFormat::NewslabPcd)) => {
            let mut reader = pcd_rs::Reader::open(input_path)?;
            let pcd_rs::PcdMeta {
                width,
                height,
                ref viewpoint,
                data,
                ..
            } = *reader.meta();

            let mut writer = pcd_rs::WriterInit {
                width,
                height,
                viewpoint: viewpoint.clone(),
                data_kind: data,
                schema: None,
            }
            .create(&output_path)?;

            reader.try_for_each(|point| -> Result<_> {
                let LibpclPoint { x, y, z, .. } = point?;
                let x = x as f64;
                let y = y as f64;
                let z = z as f64;
                let distance = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();
                let azimuthal_angle = y.atan2(x);
                let polar_angle = (x.powi(2) + y.powi(2) / z).atan();
                let vertical_angle = f64::consts::FRAC_PI_2 - polar_angle;

                let point = NewslabV1Point {
                    x,
                    y,
                    z,
                    distance,
                    azimuthal_angle,
                    vertical_angle,
                    intensity: 0.0,
                    laser_id: 0,
                    timestamp_ns: 0,
                };

                writer.push(&point)?;
                Ok(())
            })?;

            writer.finish()?;
        }
        (FileFormat::NewslabPcd, Some(FileFormat::LibpclPcd)) => {
            let mut reader = pcd_rs::Reader::open(input_path)?;
            let pcd_rs::PcdMeta {
                width,
                height,
                ref viewpoint,
                data,
                ..
            } = *reader.meta();

            let mut writer = pcd_rs::WriterInit {
                width,
                height,
                viewpoint: viewpoint.clone(),
                data_kind: data,
                schema: None,
            }
            .create(&output_path)?;

            reader.try_for_each(|point| -> Result<_> {
                let NewslabV1Point { x, y, z, .. } = point?;
                let x = x as f32;
                let y = y as f32;
                let z = z as f32;
                let point = LibpclPoint { x, y, z, rgb: 0 };

                writer.push(&point)?;
                Ok(())
            })?;

            writer.finish()?;
        }
        (FileFormat::Pcap, _) => {
            if output_format.is_some() {
                eprintln!(
                    "Warning: the output path '{}' is treated as a directory",
                    output_path.display()
                );
            }
            utils::pcap_to_pcd(input_path, output_path)?;
        }
        (FileFormat::LibpclPcd, None) | (FileFormat::NewslabPcd, None) => {
            bail!("You must specify a output file when transforming pcd/newslab-pcd");
        }

        (FileFormat::LibpclPcd, Some(FileFormat::LibpclPcd))
        | (FileFormat::NewslabPcd, Some(FileFormat::NewslabPcd)) => {
            return Ok(());
        }
        (_, Some(FileFormat::Pcap)) => {
            bail!("converting to pcap file is not supported");
        }
    }

    Ok(())
}

fn guess_file_format(file: impl AsRef<Path>) -> Option<FileFormat> {
    let file = file.as_ref();
    let file_name = file.file_name()?.to_str()?;

    let format = if file_name.ends_with(".newslab.pcd") {
        FileFormat::NewslabPcd
    } else if file_name.ends_with(".pcd") {
        FileFormat::LibpclPcd
    } else if file_name.ends_with(".pcap") {
        FileFormat::Pcap
    } else {
        return None;
    };

    Some(format)
}
