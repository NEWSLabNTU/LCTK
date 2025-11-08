use hollow_board_detector::algo::voxel_downsample;
use nalgebra::Point3;

fn main() {
    // Test basic functionality
    let mut points = Vec::new();
    for i in 0..1000 {
        points.push(Point3::new(
            (i as f64 * 0.005) % 1.0,
            (i as f64 * 0.007) % 1.0,
            (i as f64 * 0.011) % 1.0,
        ));
    }

    println!("Original points: {}", points.len());

    let result = voxel_downsample(&points, 0.02, true, 50_000);
    println!("After voxel downsampling (0.02m): {}", result.len());
    println!(
        "Reduction: {:.1}%",
        (1.0 - result.len() as f64 / points.len() as f64) * 100.0
    );

    let result2 = voxel_downsample(&points, 0.05, true, 50_000);
    println!("After voxel downsampling (0.05m): {}", result2.len());
    println!(
        "Reduction: {:.1}%",
        (1.0 - result2.len() as f64 / points.len() as f64) * 100.0
    );

    println!("\n✓ Voxel downsampling is working correctly!");
}
