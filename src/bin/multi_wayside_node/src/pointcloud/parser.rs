use crate::types::LidarPoint;
use eyre::Result;
use nalgebra::Point3;
use sensor_msgs::msg::PointCloud2;
use std::mem;

/// Trait for parsing point cloud data
pub trait PointCloudParser: Send + Sync {
    fn parse(&self, msg: &PointCloud2) -> Result<Vec<LidarPoint>>;
    fn to_nalgebra_points(&self, points: &[LidarPoint]) -> Vec<Point3<f64>>;
}

/// Default implementation of PointCloudParser
pub struct DefaultPointCloudParser;

impl PointCloudParser for DefaultPointCloudParser {
    fn parse(&self, msg: &PointCloud2) -> Result<Vec<LidarPoint>> {
        parse_pointcloud2(msg)
    }

    fn to_nalgebra_points(&self, points: &[LidarPoint]) -> Vec<Point3<f64>> {
        points
            .iter()
            .map(|p| Point3::new(p.x as f64, p.y as f64, p.z as f64))
            .collect()
    }
}

/// Parse PointCloud2 message into vector of LidarPoint
pub fn parse_pointcloud2(msg: &PointCloud2) -> Result<Vec<LidarPoint>> {
    // Validate we have the expected fields
    let x_field = msg
        .fields
        .iter()
        .find(|f| f.name == "x")
        .ok_or_else(|| eyre::eyre!("Missing 'x' field in PointCloud2"))?;
    let y_field = msg
        .fields
        .iter()
        .find(|f| f.name == "y")
        .ok_or_else(|| eyre::eyre!("Missing 'y' field in PointCloud2"))?;
    let z_field = msg
        .fields
        .iter()
        .find(|f| f.name == "z")
        .ok_or_else(|| eyre::eyre!("Missing 'z' field in PointCloud2"))?;

    // Extract offsets
    let x_offset = x_field.offset as usize;
    let y_offset = y_field.offset as usize;
    let z_offset = z_field.offset as usize;

    // Parse points
    let point_step = msg.point_step as usize;
    let num_points = (msg.data.len() / point_step) as usize;

    let mut points = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let base_offset = i * point_step;

        // Read x, y, z as f32 (assuming FLOAT32 datatype = 7)
        let x = read_f32_le(&msg.data, base_offset + x_offset);
        let y = read_f32_le(&msg.data, base_offset + y_offset);
        let z = read_f32_le(&msg.data, base_offset + z_offset);

        points.push(LidarPoint {
            x,
            y,
            z,
            intensity: 0.0,
        });
    }

    Ok(points)
}

#[inline]
fn read_f32_le(data: &[u8], offset: usize) -> f32 {
    let bytes: [u8; 4] = [
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ];
    f32::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_msgs::msg::{PointCloud2, PointField};

    fn create_test_pointcloud() -> PointCloud2 {
        let mut msg = PointCloud2::default();
        msg.height = 1;
        msg.width = 3;
        msg.point_step = 12; // 3 floats * 4 bytes

        // Define fields
        msg.fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: 7, // FLOAT32
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];

        // Add test data
        let points = vec![
            (1.0f32, 2.0f32, 3.0f32),
            (4.0f32, 5.0f32, 6.0f32),
            (7.0f32, 8.0f32, 9.0f32),
        ];

        msg.data = Vec::with_capacity(points.len() * 12);
        for (x, y, z) in points {
            msg.data.extend_from_slice(&x.to_le_bytes());
            msg.data.extend_from_slice(&y.to_le_bytes());
            msg.data.extend_from_slice(&z.to_le_bytes());
        }

        msg.row_step = msg.data.len() as u32;
        msg
    }

    #[test]
    fn test_parse_pointcloud2() {
        let msg = create_test_pointcloud();
        let parser = DefaultPointCloudParser;
        let points = parser.parse(&msg).unwrap();

        assert_eq!(points.len(), 3);
        assert_eq!(points[0].x, 1.0);
        assert_eq!(points[0].y, 2.0);
        assert_eq!(points[0].z, 3.0);
        assert_eq!(points[2].x, 7.0);
        assert_eq!(points[2].y, 8.0);
        assert_eq!(points[2].z, 9.0);
    }

    #[test]
    fn test_to_nalgebra_points() {
        let parser = DefaultPointCloudParser;
        let lidar_points = vec![
            LidarPoint {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 0.0,
            },
            LidarPoint {
                x: 4.0,
                y: 5.0,
                z: 6.0,
                intensity: 0.0,
            },
        ];

        let nalgebra_points = parser.to_nalgebra_points(&lidar_points);
        assert_eq!(nalgebra_points.len(), 2);
        assert_eq!(nalgebra_points[0].x, 1.0);
        assert_eq!(nalgebra_points[1].z, 6.0);
    }
}
