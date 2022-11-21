use anyhow::{Context, Result};
use console::Term;
use dialoguer::{Input, Select};
use iterator_ext::IteratorExt;
use lidar_utils::velodyne::{self, FrameConverter as _};
use num_traits::ToPrimitive;
use pcd_format::LibpclPoint;
use std::{fs, iter, path::Path};

use crate::types::{FileFormat, LidarType};

pub fn save_pcd<P, T>(points: Vec<[T; 3]>, pcd_file: P, data_kind: pcd_rs::DataKind) -> Result<()>
where
    P: AsRef<Path>,
    T: ToPrimitive,
{
    let mut writer = pcd_rs::WriterInit {
        width: points.len() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind,
        schema: None,
    }
    .create(pcd_file)?;

    points.into_iter().try_for_each(|[x, y, z]| -> Result<_> {
        let point = LibpclPoint {
            x: x.to_f32().unwrap(),
            y: y.to_f32().unwrap(),
            z: z.to_f32().unwrap(),
            rgb: 0,
        };
        writer.push(&point)?;
        Ok(())
    })?;

    writer.finish()?;

    Ok(())
}

pub fn pcap_to_pcd<I, O>(input_file: I, output_dir: O) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let input_file = input_file.as_ref();
    let output_dir = output_dir.as_ref();

    // Get some user informations
    let term = Term::stdout();
    let lidar_type = {
        let choice = Select::new()
            .with_prompt("What kind of lidar was used to collect this pcap?")
            .items(&["Vlp16", "Vlp32"])
            .default(1)
            .interact_on(&term)?;

        match choice {
            0 => LidarType::Vlp16,
            1 => LidarType::Vlp32,
            _ => unreachable!(),
        }
    };
    let target_type = {
        let choice = Select::new()
            .with_prompt("What type of file do you want to convert?")
            .items(&["standard pcd", "newslab pcd"])
            .default(0)
            .interact_on(&term)?;

        match choice {
            0 => FileFormat::LibpclPcd,
            1 => FileFormat::NewslabPcd,
            _ => unreachable!(),
        }
    };
    let start_number: usize = Input::new()
        .with_prompt("Which frame number do you want to transform from?")
        .default(0)
        .interact()?;
    let number_of_frames: usize = Input::new()
        .with_prompt("How many frames do you want to transform (0 for all)?")
        .default(0)
        .interact()?;

    // prepare pcap capture handlers
    let mut cap = pcap::Capture::from_file(input_file)
        .with_context(|| format!("unable to open file '{}'", input_file.display()))?;
    let mut frame_converter = match lidar_type {
        LidarType::Vlp16 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::puck_hires_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
        LidarType::Vlp32 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::vlp_32c_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
    };

    fs::create_dir_all(output_dir)
        .with_context(|| format!("unable to create directory {}", output_dir.display()))?;

    match target_type {
        FileFormat::LibpclPcd => {
            use velodyne::DynamicReturnPoints as DP;

            // Create a packet iterator
            let packet_iter = iter::from_fn(|| -> Option<Option<_>> {
                let raw_packet = cap.next().ok()?;
                let velodyne_packet = velodyne::DataPacket::from_pcap(&raw_packet).ok();
                Some(velodyne_packet)
            })
            .flatten();

            // Cconvert packets to frames
            let frame_iter = packet_iter
                .map(Ok)
                .try_flat_map(|packet| frame_converter.convert::<velodyne::DataPacket>(packet))
                .enumerate()
                .map(|(index, frame)| anyhow::Ok((index, frame?)));

            // Restrict the range of frame indices
            let frame_iter = frame_iter.skip(start_number);
            let mut frame_iter: Box<dyn Iterator<Item = Result<(usize, DP)>> + Sync + Send> =
                if number_of_frames > 0 {
                    Box::new(frame_iter.take(number_of_frames))
                } else {
                    Box::new(frame_iter)
                };

            frame_iter.try_for_each(|args| -> Result<_> {
                use uom::si::length::meter;

                let (index, frame) = args?;
                println!("transforming frame number {}", index);

                let points: Vec<_> = match frame {
                    DP::Single(points) => points
                        .into_iter()
                        .map(|point| point.data.position)
                        .map(|[x, y, z]| [x.get::<meter>(), y.get::<meter>(), z.get::<meter>()])
                        .collect(),
                    DP::Dual(points) => points
                        .into_iter()
                        .map(|point| point.strongest_return_data.position)
                        .map(|[x, y, z]| [x.get::<meter>(), y.get::<meter>(), z.get::<meter>()])
                        .collect(),
                };

                let pcd_file = output_dir.join(format!("{:06}.pcd", index));
                save_pcd(points, &pcd_file, pcd_rs::DataKind::Ascii).with_context(|| {
                    format!("failed to create the pcd file '{}'", pcd_file.display())
                })?;

                Ok(())
            })?;
        }
        FileFormat::NewslabPcd => {
            todo!()
        }
        _ => unreachable!(),
    }
    Ok(())
}

pub fn time(file: impl AsRef<Path>) -> Result<()> {
    let term = Term::stdout();
    let lidar_type = {
        let choice = Select::new()
            .with_prompt("What kind of lidar was used to collect this pcap?")
            .items(&["Vlp16", "Vlp32"])
            .default(1)
            .interact_on(&term)?;

        match choice {
            0 => LidarType::Vlp16,
            1 => LidarType::Vlp32,
            _ => unreachable!(),
        }
    };

    // prepare pcap capture handlers
    let mut cap = pcap::Capture::from_file(file)?;
    let mut frame_converter = match lidar_type {
        LidarType::Vlp16 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::puck_hires_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
        LidarType::Vlp32 => velodyne::Dynamic_FrameConverter::from_config(
            velodyne::Config::vlp_32c_dynamic_return(velodyne::ReturnMode::StrongestReturn)
                .into_dyn(),
        ),
    };

    iter::from_fn(|| {
        let packet = cap.next().ok()?;
        let packet = velodyne::DataPacket::from_pcap(&packet).ok();
        Some(packet)
    })
    .flatten()
    .map(Ok)
    .try_flat_map(|packet| frame_converter.convert::<velodyne::DataPacket>(packet))
    .enumerate()
    .map(|(index, frame)| -> Result<_> { Ok((index, frame?)) })
    .try_for_each(|x| -> Result<()> {
        let (idx, frame) = x?;
        let device_timestamp = frame
            .into_iter()
            .map(|point| {
                let nanos = point.timestamp().get::<uom::si::time::nanosecond>() as u64;
                std::time::Duration::from_nanos(nanos)
            })
            .min()
            .unwrap();
        println!(
            "Frame: {}, device_time: {:>10.4} (sec)",
            idx,
            device_timestamp.as_secs_f64()
        );
        Ok(())
    })?;
    Ok(())
}
