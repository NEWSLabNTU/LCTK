use crate::types::LidarPoint;
use nalgebra::Point3;

/// Trait for filtering point cloud data
pub trait PointCloudFilter: Send + Sync {
    fn filter(&self, points: &[LidarPoint]) -> Vec<LidarPoint>;
    fn filter_nalgebra(&self, points: &[Point3<f64>]) -> Vec<Point3<f64>>;
}

/// Range-based point cloud filter
pub struct RangeFilter {
    min_range: f32,
    max_range: f32,
}

impl RangeFilter {
    pub fn new(min_range: f32, max_range: f32) -> Self {
        Self {
            min_range,
            max_range,
        }
    }
}

impl PointCloudFilter for RangeFilter {
    fn filter(&self, points: &[LidarPoint]) -> Vec<LidarPoint> {
        points
            .iter()
            .filter(|p| {
                let range = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
                range >= self.min_range && range <= self.max_range
            })
            .cloned()
            .collect()
    }

    fn filter_nalgebra(&self, points: &[Point3<f64>]) -> Vec<Point3<f64>> {
        points
            .iter()
            .filter(|p| {
                let range = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
                range >= self.min_range as f64 && range <= self.max_range as f64
            })
            .cloned()
            .collect()
    }
}

/// Composite filter that applies multiple filters in sequence
pub struct CompositeFilter {
    filters: Vec<Box<dyn PointCloudFilter>>,
}

impl CompositeFilter {
    pub fn new() -> Self {
        Self {
            filters: Vec::new(),
        }
    }

    pub fn add_filter(mut self, filter: Box<dyn PointCloudFilter>) -> Self {
        self.filters.push(filter);
        self
    }
}

impl PointCloudFilter for CompositeFilter {
    fn filter(&self, points: &[LidarPoint]) -> Vec<LidarPoint> {
        let mut result = points.to_vec();
        for filter in &self.filters {
            result = filter.filter(&result);
        }
        result
    }

    fn filter_nalgebra(&self, points: &[Point3<f64>]) -> Vec<Point3<f64>> {
        let mut result = points.to_vec();
        for filter in &self.filters {
            result = filter.filter_nalgebra(&result);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_range_filter() {
        let filter = RangeFilter::new(1.0, 5.0);
        let points = vec![
            LidarPoint {
                x: 0.5,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 0.5, filtered out
            LidarPoint {
                x: 2.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 2.0, kept
            LidarPoint {
                x: 3.0,
                y: 4.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 5.0, kept
            LidarPoint {
                x: 6.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 6.0, filtered out
        ];

        let filtered = filter.filter(&points);
        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].x, 2.0);
        assert_eq!(filtered[1].x, 3.0);
    }

    #[test]
    fn test_composite_filter() {
        let filter = CompositeFilter::new()
            .add_filter(Box::new(RangeFilter::new(1.0, 10.0)))
            .add_filter(Box::new(RangeFilter::new(2.0, 5.0)));

        let points = vec![
            LidarPoint {
                x: 0.5,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 0.5
            LidarPoint {
                x: 1.5,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 1.5
            LidarPoint {
                x: 3.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 3.0
            LidarPoint {
                x: 6.0,
                y: 0.0,
                z: 0.0,
                intensity: 0.0,
            }, // range = 6.0
        ];

        let filtered = filter.filter(&points);
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].x, 3.0);
    }
}
