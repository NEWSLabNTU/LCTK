use crate::types::RoiBounds;
use eyre::Result;
use nalgebra::Point3;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

/// Trait for managing ROI (Region of Interest) bounds
pub trait RoiManager: Send + Sync {
    fn get_bounds(&self, lidar_id: u8) -> Option<RoiBounds>;
    fn set_bounds(&self, lidar_id: u8, bounds: RoiBounds) -> Result<()>;
    fn apply_crop(&self, points: &[Point3<f64>], lidar_id: u8) -> Vec<Point3<f64>>;
    fn get_all_bounds(&self) -> HashMap<u8, RoiBounds>;
}

/// Default implementation of RoiManager
pub struct DefaultRoiManager {
    bounds: Arc<Mutex<HashMap<u8, RoiBounds>>>,
}

impl DefaultRoiManager {
    pub fn new() -> Self {
        Self {
            bounds: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn with_initial_bounds(initial_bounds: HashMap<u8, RoiBounds>) -> Self {
        Self {
            bounds: Arc::new(Mutex::new(initial_bounds)),
        }
    }
}

impl RoiManager for DefaultRoiManager {
    fn get_bounds(&self, lidar_id: u8) -> Option<RoiBounds> {
        self.bounds.lock().unwrap().get(&lidar_id).cloned()
    }

    fn set_bounds(&self, lidar_id: u8, bounds: RoiBounds) -> Result<()> {
        self.bounds.lock().unwrap().insert(lidar_id, bounds);
        Ok(())
    }

    fn apply_crop(&self, points: &[Point3<f64>], lidar_id: u8) -> Vec<Point3<f64>> {
        if let Some(bounds) = self.get_bounds(lidar_id) {
            crate::types::apply_roi_crop(points, &bounds)
        } else {
            points.to_vec()
        }
    }

    fn get_all_bounds(&self) -> HashMap<u8, RoiBounds> {
        self.bounds.lock().unwrap().clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roi_manager_basic_operations() {
        let manager = DefaultRoiManager::new();

        // Initially no bounds
        assert!(manager.get_bounds(1).is_none());

        // Set bounds
        let bounds = RoiBounds {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: -1.0,
            max_z: 1.0,
        };
        manager.set_bounds(1, bounds.clone()).unwrap();

        // Get bounds
        let retrieved = manager.get_bounds(1).unwrap();
        assert_eq!(retrieved.min_x, bounds.min_x);
        assert_eq!(retrieved.max_x, bounds.max_x);
    }

    #[test]
    fn test_roi_manager_cropping() {
        let mut initial_bounds = HashMap::new();
        initial_bounds.insert(
            1,
            RoiBounds {
                min_x: -1.0,
                max_x: 1.0,
                min_y: -1.0,
                max_y: 1.0,
                min_z: -1.0,
                max_z: 1.0,
            },
        );

        let manager = DefaultRoiManager::with_initial_bounds(initial_bounds);

        let points = vec![
            Point3::new(0.0, 0.0, 0.0),    // inside
            Point3::new(0.5, 0.5, 0.5),    // inside
            Point3::new(2.0, 0.0, 0.0),    // outside
            Point3::new(0.0, 2.0, 0.0),    // outside
            Point3::new(-0.5, -0.5, -0.5), // inside
        ];

        let cropped = manager.apply_crop(&points, 1);
        assert_eq!(cropped.len(), 3);

        // No bounds for lidar 2, should return all points
        let cropped = manager.apply_crop(&points, 2);
        assert_eq!(cropped.len(), 5);
    }

    #[test]
    fn test_roi_manager_multiple_lidars() {
        let manager = DefaultRoiManager::new();

        let bounds1 = RoiBounds {
            min_x: -1.0,
            max_x: 1.0,
            min_y: -1.0,
            max_y: 1.0,
            min_z: -1.0,
            max_z: 1.0,
        };

        let bounds2 = RoiBounds {
            min_x: -2.0,
            max_x: 2.0,
            min_y: -2.0,
            max_y: 2.0,
            min_z: -2.0,
            max_z: 2.0,
        };

        manager.set_bounds(1, bounds1).unwrap();
        manager.set_bounds(2, bounds2).unwrap();

        let all_bounds = manager.get_all_bounds();
        assert_eq!(all_bounds.len(), 2);
        assert!(all_bounds.contains_key(&1));
        assert!(all_bounds.contains_key(&2));
    }
}
