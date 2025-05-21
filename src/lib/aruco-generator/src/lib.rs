use anyhow::{ensure, Result};
use aruco_config::{ArucoDictionary, MultiArucoPattern};
use console::Term;
use dialoguer::{Confirm, Input, Select};
use indexmap::IndexSet;
use measurements::Length;
use noisy_float::prelude::*;
use opencv::{
    aruco::CharucoBoard,
    core::{prelude::*, Size, Vector, CV_8UC1},
    highgui, imgcodecs,
    prelude::*,
};
use rand::prelude::*;
use std::io::prelude::*;
use strum::VariantNames;

pub mod config;
pub use config::*;

const MILLIMETERS_PER_INCH: f64 = 25.4;

/// Core functionality for generating ArUco markers and boards
pub struct ArucoGenerator;

impl ArucoGenerator {
    /// Generate ArUco image from configuration
    pub fn generate_from_config(config: &Config, preview: bool) -> Result<()> {
        config.validate()?;

        match config {
            Config::SingleAruco(_config) => {
                // TODO: Implement single ArUco generation
                todo!("Single ArUco generation not yet implemented");
            }
            Config::SingleCharuco(config) => {
                Self::generate_single_charuco_image(config, preview)?;
            }
            Config::MultipleArucos(config) => {
                Self::generate_multiple_arucos_image(config, preview)?;
            }
        }
        Ok(())
    }

    /// Generate a single ChArUco board image
    pub fn generate_single_charuco_image(
        config: &SingleCharucoConfig,
        preview: bool,
    ) -> Result<()> {
        let dictionary = &config.dictionary;
        let squares_per_side = config.squares_per_side as i32;
        let border_bits = config.border_bits as i32;
        let marker_to_square_length_ratio = config.marker_to_square_length_ratio;
        let paper_size_mm = config.paper_size_mm;
        let margin_size_mm = config.margin_size_mm;
        let dpi = config.dpi;
        let output_path = &config.output_path;

        let mut board = {
            let square_length = (paper_size_mm - margin_size_mm * 2.0) / squares_per_side as f64;
            let marker_length = square_length * marker_to_square_length_ratio;
            println!("square length = {} mm", square_length);
            println!("marker length = {} mm", marker_length);

            CharucoBoard::create(
                squares_per_side,
                squares_per_side,
                square_length as f32,
                marker_length as f32,
                &dictionary.to_opencv_dictionary()?,
            )?
        };

        let image = {
            let image_size_pixels = (paper_size_mm / MILLIMETERS_PER_INCH * dpi) as i32;
            let margin_size_pixels = (margin_size_mm / MILLIMETERS_PER_INCH * dpi) as i32;
            println!("image size = {} pixels", image_size_pixels);
            println!("margin size = {} pixels", margin_size_pixels);

            let mut image = Mat::zeros(image_size_pixels, image_size_pixels, CV_8UC1)?.to_mat()?;
            board.draw(
                Size::new(image_size_pixels, image_size_pixels),
                &mut image,
                margin_size_pixels,
                border_bits,
            )?;
            image
        };

        imgcodecs::imwrite(output_path, &image, &Vector::<i32>::new())?;
        println!("Saved ChArUco board to: {}", output_path);

        if preview {
            highgui::imshow("preview", &image)?;
            highgui::wait_key(0)?;
        }

        Ok(())
    }

    /// Generate a multiple ArUco markers image
    pub fn generate_multiple_arucos_image(
        config: &MultipleArucosConfig,
        preview: bool,
    ) -> Result<()> {
        let dictionary = &config.dictionary;
        let opencv_dictionary = dictionary.to_opencv_dictionary()?;
        let num_squares_per_side = config.num_squares_per_side;
        let board_size_mm = config.board_size_mm;
        let board_border_size_mm = config.board_border_size_mm;
        let marker_square_size_ratio = config.marker_square_size_ratio;
        let border_bits = config.border_bits;
        let dpi = config.dpi;

        let n_markers = num_squares_per_side.pow(2) as usize;
        let dict_size = opencv_dictionary.bytes_list().rows() as usize;

        let marker_ids = match &config.marker_ids {
            MarkerIds::Random => {
                let mut rng = rand::thread_rng();
                let mut id_set = IndexSet::new();

                while id_set.len() < n_markers {
                    let rand_id = rng.gen_range(0..(dict_size as u32));
                    id_set.insert(rand_id);
                }

                let id_vec: Vec<_> = id_set.into_iter().collect();
                println!(
                    "Generated random marker IDs: {}",
                    id_vec
                        .iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                );
                id_vec
            }
            MarkerIds::Specific { ids } => {
                println!(
                    "Using specified marker IDs: {}",
                    ids.iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                );
                ids.clone()
            }
        };

        let pattern = MultiArucoPattern {
            marker_ids: marker_ids.clone(),
            dictionary: *dictionary,
            board_size: Length::from_millimeters(board_size_mm),
            board_border_size: Length::from_millimeters(board_border_size_mm),
            marker_square_size_ratio: r64(marker_square_size_ratio),
            num_squares_per_side,
            border_bits,
        };

        let output_path = config.output_path.clone().unwrap_or_else(|| {
            let marker_ids_string = marker_ids
                .iter()
                .map(|id| id.to_string())
                .collect::<Vec<_>>()
                .join(",");

            format!(
                "{dictionary}\
                 -\
                 {num_squares_per_side}\
                 x\
                 {num_squares_per_side}\
                 -\
                 {board_size_mm}\
                 -\
                 {board_border_size_mm}\
                 -\
                 {marker_square_size_ratio}\
                 -\
                 {marker_ids_string}\
                 .jpg",
            )
        });

        println!(
            "square size = {} mm",
            pattern.square_size().as_millimeters()
        );
        println!(
            "marker size = {} mm",
            pattern.marker_size().as_millimeters()
        );

        let image = pattern.to_opencv_mat(dpi)?;

        // save image
        imgcodecs::imwrite(&output_path, &image, &Vector::<i32>::new())?;
        println!("Saved multiple ArUco pattern to: {}", output_path);

        if preview {
            highgui::imshow("preview", &image)?;
            highgui::wait_key(0)?;
        }

        Ok(())
    }
}

/// Interactive configuration builder for console applications
pub struct InteractiveBuilder;

impl InteractiveBuilder {
    /// Create configuration through interactive prompts
    pub fn build_interactive_config() -> Result<Config> {
        let mut term = Term::stdout();

        let choice = Select::new()
            .with_prompt("What is the kind of marker?")
            .items(&["single ArUco", "single ChArUco", "multiple ArUcos"])
            .default(2)
            .interact_on(&term)?;

        let config = match choice {
            0 => Self::interact_single_aruco(&mut term)?,
            1 => Self::interact_single_charuco(&mut term)?,
            2 => Self::interact_multiple_arucos(&mut term)?,
            _ => unreachable!(),
        };

        Ok(config)
    }

    fn interact_single_aruco(_term: &mut Term) -> Result<Config> {
        // TODO: Implement single ArUco interactive configuration
        todo!("Single ArUco interactive configuration not yet implemented");
    }

    fn interact_single_charuco(term: &mut Term) -> Result<Config> {
        let dictionary = Self::query_dictionary(term)?;

        let squares_per_side = Input::<u16>::new()
            .with_prompt("Number of squares per side?")
            .default(2)
            .interact_on(term)?;
        ensure!(squares_per_side > 0);

        let border_bits = Input::<u16>::new()
            .with_prompt("Border bits?")
            .default(1)
            .interact_on(term)?;
        ensure!(border_bits > 0);

        let marker_to_square_length_ratio = Input::<f64>::new()
            .with_prompt("Marker to square size ratio?")
            .default(0.8)
            .interact_on(term)?;
        ensure!(marker_to_square_length_ratio > 0.0 && marker_to_square_length_ratio < 1.0);

        let paper_size_mm = Input::<f64>::new()
            .with_prompt("Paper size (millimeters)?")
            .default(500.0)
            .interact_on(term)?;
        ensure!(paper_size_mm > 0.0);

        let margin_size_mm = Input::<f64>::new()
            .with_prompt("Margin size (millimeters)?")
            .default(50.0)
            .interact_on(term)?;
        ensure!(margin_size_mm * 2.0 < paper_size_mm);

        let dpi = Input::<f64>::new()
            .with_prompt("Pixels per inch (dpi)?")
            .default(300.0)
            .interact_on(term)?;
        ensure!(dpi > 0.0);

        let output_path = Input::<String>::new()
            .with_prompt("save to?")
            .interact_on(term)?;

        Ok(Config::SingleCharuco(SingleCharucoConfig {
            dictionary,
            squares_per_side,
            border_bits,
            marker_to_square_length_ratio,
            paper_size_mm,
            margin_size_mm,
            dpi,
            output_path,
        }))
    }

    fn interact_multiple_arucos(term: &mut Term) -> Result<Config> {
        let dictionary = Self::query_dictionary(term)?;
        let opencv_dictionary = dictionary.to_opencv_dictionary()?;

        let num_squares_per_side = Input::<u32>::new()
            .with_prompt("How many squares per side?")
            .default(2)
            .interact_on(term)?;
        ensure!(num_squares_per_side >= 1);

        let board_size_mm = Input::<f64>::new()
            .with_prompt("Board size (millimeters)?")
            .default(500.0)
            .interact_on(term)?;
        ensure!(board_size_mm > 0.0);

        let board_border_size_mm = Input::<f64>::new()
            .with_prompt("Board border size (millimeters)?")
            .default(10.0)
            .interact_on(term)?;
        ensure!(board_border_size_mm >= 0.0 && board_size_mm - board_border_size_mm * 2.0 > 0.0);

        let marker_square_size_ratio = Input::<f64>::new()
            .with_prompt("Marker size to square size ratio?")
            .default(0.8)
            .interact_on(term)?;
        ensure!(marker_square_size_ratio > 0.0 && marker_square_size_ratio < 1.0);

        let border_bits = Input::<u32>::new()
            .with_prompt("Border bits?")
            .default(1)
            .interact_on(term)?;
        ensure!(border_bits > 0);

        let dpi = Input::<f64>::new()
            .with_prompt("Pixels per inch (dpi)?")
            .default(300.0)
            .interact_on(term)?;
        ensure!(dpi > 0.0);

        let n_markers = num_squares_per_side.pow(2) as usize;
        let dict_size = opencv_dictionary.bytes_list().rows() as usize;
        let marker_ids_vec = Self::query_marker_ids(term, n_markers, dict_size)?;

        let marker_ids = MarkerIds::Specific {
            ids: marker_ids_vec.clone(),
        };

        let default_output_path = {
            let marker_ids_string = marker_ids_vec
                .iter()
                .map(|id| id.to_string())
                .collect::<Vec<_>>()
                .join(",");

            format!(
                "{dictionary}\
                 -\
                 {num_squares_per_side}\
                 x\
                 {num_squares_per_side}\
                 -\
                 {board_size_mm}\
                 -\
                 {board_border_size_mm}\
                 -\
                 {marker_square_size_ratio}\
                 -\
                 {marker_ids_string}\
                 .jpg",
            )
        };

        let output_path = Input::<String>::new()
            .with_prompt("save to?")
            .default(default_output_path)
            .interact_on(term)?;

        Ok(Config::MultipleArucos(MultipleArucosConfig {
            dictionary,
            num_squares_per_side,
            board_size_mm,
            board_border_size_mm,
            marker_square_size_ratio,
            border_bits,
            dpi,
            marker_ids,
            output_path: Some(output_path),
        }))
    }

    fn query_dictionary(term: &Term) -> Result<ArucoDictionary> {
        let names = ArucoDictionary::VARIANTS;

        let choice = Select::new()
            .with_prompt("Which dictionary to use?")
            .items(names)
            .default(7)
            .interact_on(term)?;

        let dict = ArucoDictionary::from_repr(choice as u8).unwrap();
        Ok(dict)
    }

    fn query_marker_ids(term: &mut Term, n_markers: usize, dict_size: usize) -> Result<Vec<u32>> {
        let use_random_ids = Confirm::new()
            .with_prompt("Generate random marker IDs?")
            .default(true)
            .interact_on(term)?;

        let marker_ids = if use_random_ids {
            let mut rng = rand::thread_rng();
            let mut id_set = IndexSet::new();

            while id_set.len() < n_markers {
                let rand_id = rng.gen_range(0..(dict_size as u32));
                id_set.insert(rand_id);
            }

            let id_vec: Vec<_> = id_set.into_iter().collect();

            writeln!(
                term,
                "{}",
                id_vec
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )?;

            id_vec
        } else {
            loop {
                let tokens_result: Result<IndexSet<_>, _> = Input::<String>::new()
                    .with_prompt("Marker IDs?")
                    .interact_on(term)?
                    .split(',')
                    .map(|token| token.parse())
                    .collect();

                let tokens = match tokens_result {
                    Ok(tokens) => tokens,
                    Err(error) => {
                        writeln!(term, "input not understood: {:?}", error)?;
                        continue;
                    }
                };

                if tokens.len() != n_markers {
                    writeln!(term, "expect {} IDs", n_markers)?;
                    continue;
                }

                let tokens: Vec<_> = tokens.into_iter().collect();

                break tokens;
            }
        };

        Ok(marker_ids)
    }
}
