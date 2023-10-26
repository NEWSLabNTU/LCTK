use crate::types::{FileFormat, LidarType};
use anyhow::{Context, Result};
use console::Term;
use dialoguer::Select;
use num_traits::ToPrimitive;
use pcd_format::LibpclPoint;
use std::{fs, path::Path};

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

pub fn pcap_to_pcd<I, O>(input_file: I, output_dir: O, start_frame: usize, num: usize) -> Result<()>
where
    I: AsRef<Path>,
    O: AsRef<Path>,
{
    let input_file = input_file.as_ref();
    let output_dir = output_dir.as_ref();

    // Get some user informations
    let lidar_type = LidarType::Vlp32;
    let target_type = FileFormat::LibpclPcd;
    //Only pick the frame begin with start_number of frame
    let start_number = start_frame;
    // let number_of_frames = num;
    //let term = Term::stdout();
    //let lidar_type = {
    //    let choice = Select::new()
    //        .with_prompt("What kind of lidar was used to collect this pcap?")
    //        .items(&["Vlp16", "Vlp32"])
    //        .default(1)
    //        .interact_on(&term)?;

    //    match choice {
    //        0 => LidarType::Vlp16,
    //        1 => LidarType::Vlp32,
    //        _ => unreachable!(),
    //    }
    //};
    //let target_type = {
    //    let choice = Select::new()
    //        .with_prompt("What type of file do you want to convert?")
    //        .items(&["standard pcd", "newslab pcd"])
    //        .default(0)
    //        .interact_on(&term)?;

    //    match choice {
    //        0 => FileFormat::LibpclPcd,
    //        1 => FileFormat::NewslabPcd,
    //        _ => unreachable!(),
    //    }
    //};
    // let start_number: usize = Input::new()
    //     .with_prompt("Which frame number do you want to transform from?")
    //     .default(1)
    //     .interact()?;
    // let number_of_frames: usize = Input::new()
    //     .with_prompt("How many frames do you want to transform (0 for all)?")
    //     .default(1)
    //     .interact()?;

    fs::create_dir_all(output_dir)
        .with_context(|| format!("unable to create directory {}", output_dir.display()))?;

    match target_type {
        FileFormat::LibpclPcd => {
            let config = match lidar_type {
                LidarType::Vlp16 => velodyne_lidar::Config::new_puck_hires_strongest(),
                LidarType::Vlp32 => velodyne_lidar::Config::new_vlp_32c_strongest(),
            };

            velodyne_lidar::iter::frame_xyz_iter_from_file(config, input_file)?
                .enumerate()
                .filter(|(index, frame)| *index >= start_number && *index < start_number + num)
                .try_for_each(|(index, frame)| -> Result<_> {
                    let frame = frame?;
                    println!("transforming frame number {}",index);
                    // if(index%10==0){
                    //     println!("transforming frame number {}~{}",index-9, index);
                    // }
                    let points: Vec<_> = frame
                        .into_point_iter()
                        .map(|point| {
                            let point = point.try_into_single().unwrap();
                            let [x, y, z] = point.measurement.xyz;
                            [x.as_meters(), y.as_meters(), z.as_meters()]
                        })
                        .collect();

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

    let config = match lidar_type {
        LidarType::Vlp16 => velodyne_lidar::Config::new_puck_hires_strongest(),
        LidarType::Vlp32 => velodyne_lidar::Config::new_vlp_32c_strongest(),
    };
    let frame_iter = velodyne_lidar::iter::frame_xyz_iter_from_file(config, file)?;

    frame_iter
        .enumerate()
        .try_for_each(|(frame_idx, frame)| -> Result<()> {
            let frame = frame?;
            let time = frame.firing_iter().next().unwrap().time();

            println!(
                "Frame: {frame_idx}, device_time: {:>10.4} (sec)",
                time.as_secs_f64()
            );
            Ok(())
        })?;
    Ok(())
}
