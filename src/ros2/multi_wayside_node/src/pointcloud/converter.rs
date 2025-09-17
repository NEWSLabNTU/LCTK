use crate::types::LidarPoint;
use nalgebra::Point3;
use sensor_msgs::msg::{PointCloud2, PointField};
use std_msgs::msg::Header;

/// Convert internal point representation to PointCloud2 message
pub fn to_pointcloud2(
    points: &[LidarPoint],
    header: Header,
    color: Option<[u8; 3]>,
) -> PointCloud2 {
    let mut msg = PointCloud2 {
        header,
        height: 1,
        width: points.len() as u32,
        is_dense: true,
        is_bigendian: false,
        ..Default::default()
    };

    // Define fields based on whether we include color
    if color.is_some() {
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
            PointField {
                name: "rgb".to_string(),
                offset: 12,
                datatype: 7, // FLOAT32 (packed RGB)
                count: 1,
            },
        ];
        msg.point_step = 16; // 4 floats * 4 bytes
    } else {
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
        msg.point_step = 12; // 3 floats * 4 bytes
    }

    // Allocate data
    msg.data = Vec::with_capacity(points.len() * msg.point_step as usize);

    // Pack points
    for point in points {
        msg.data.extend_from_slice(&point.x.to_le_bytes());
        msg.data.extend_from_slice(&point.y.to_le_bytes());
        msg.data.extend_from_slice(&point.z.to_le_bytes());

        if let Some([r, g, b]) = color {
            // Pack RGB into float
            let rgb = ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);
            let rgb_float = f32::from_bits(rgb);
            msg.data.extend_from_slice(&rgb_float.to_le_bytes());
        }
    }

    msg.row_step = msg.data.len() as u32;
    msg
}

/// Convert nalgebra points to PointCloud2 message
pub fn nalgebra_to_pointcloud2(
    points: &[Point3<f64>],
    header: Header,
    color: Option<[u8; 3]>,
) -> PointCloud2 {
    let lidar_points: Vec<LidarPoint> = points
        .iter()
        .map(|p| LidarPoint {
            x: p.x as f32,
            y: p.y as f32,
            z: p.z as f32,
            intensity: 0.0,
        })
        .collect();
    to_pointcloud2(&lidar_points, header, color)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_pointcloud2_without_color() {
        let points = vec![
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

        let header = Header::default();
        let msg = to_pointcloud2(&points, header, None);

        assert_eq!(msg.width, 2);
        assert_eq!(msg.height, 1);
        assert_eq!(msg.point_step, 12);
        assert_eq!(msg.fields.len(), 3);
        assert_eq!(msg.data.len(), 24); // 2 points * 12 bytes
    }

    #[test]
    fn test_to_pointcloud2_with_color() {
        let points = vec![LidarPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        }];

        let header = Header::default();
        let msg = to_pointcloud2(&points, header, Some([255, 0, 0]));

        assert_eq!(msg.width, 1);
        assert_eq!(msg.point_step, 16);
        assert_eq!(msg.fields.len(), 4);
        assert_eq!(msg.fields[3].name, "rgb");
        assert_eq!(msg.data.len(), 16); // 1 point * 16 bytes
    }

    #[test]
    fn test_nalgebra_to_pointcloud2() {
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];

        let header = Header::default();
        let msg = nalgebra_to_pointcloud2(&points, header, None);

        assert_eq!(msg.width, 2);
        assert_eq!(msg.height, 1);
        assert_eq!(msg.point_step, 12);
    }
}
