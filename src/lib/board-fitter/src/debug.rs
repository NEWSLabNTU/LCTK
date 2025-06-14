//! Debug infrastructure for board-fitter library
//!
//! This module provides comprehensive debugging features through a callback-based architecture
//! that allows runtime configuration without recompilation. The debug system is designed to
//! have zero runtime overhead when disabled.

use std::{
    collections::HashMap,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};

use crate::types::{BoardDetection, DetectedHole, DetectedPlane, PointCloud};

/// Debug verbosity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugVerbosity {
    /// No debug output
    None,
    /// Summary statistics only
    Summary,
    /// Detailed metrics and intermediate results
    Detailed,
    /// Verbose output with all available debug information
    Verbose,
}

impl Default for DebugVerbosity {
    fn default() -> Self {
        DebugVerbosity::None
    }
}

/// Configuration for debug output
#[derive(Debug, Clone)]
pub struct DebugConfig {
    /// Enable/disable performance timing measurements
    pub timing_enabled: bool,
    /// List of pipeline stages to capture outputs from
    pub output_stages: Vec<String>,
    /// Optional directory for file outputs
    pub output_directory: Option<PathBuf>,
    /// Debug verbosity level
    pub verbosity_level: DebugVerbosity,
    /// Maximum number of intermediate point clouds to store
    pub max_point_clouds: usize,
    /// Whether to enable memory usage tracking
    pub memory_tracking: bool,
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            timing_enabled: false,
            output_stages: Vec::new(),
            output_directory: None,
            verbosity_level: DebugVerbosity::None,
            max_point_clouds: 10,
            memory_tracking: false,
        }
    }
}

/// Builder for DebugConfig with fluent API
#[derive(Debug, Default)]
pub struct DebugConfigBuilder {
    config: DebugConfig,
}

impl DebugConfigBuilder {
    /// Create a new debug config builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable timing measurements
    pub fn with_timing(mut self) -> Self {
        self.config.timing_enabled = true;
        self
    }

    /// Add a pipeline stage to capture outputs from
    pub fn capture_stage<S: Into<String>>(mut self, stage: S) -> Self {
        self.config.output_stages.push(stage.into());
        self
    }

    /// Capture outputs from multiple stages
    pub fn capture_stages<I, S>(mut self, stages: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        self.config
            .output_stages
            .extend(stages.into_iter().map(|s| s.into()));
        self
    }

    /// Set output directory for file-based debugging
    pub fn output_dir<P: Into<PathBuf>>(mut self, path: P) -> Self {
        self.config.output_directory = Some(path.into());
        self
    }

    /// Set debug verbosity level
    pub fn verbosity(mut self, level: DebugVerbosity) -> Self {
        self.config.verbosity_level = level;
        self
    }

    /// Set maximum number of point clouds to store
    pub fn max_point_clouds(mut self, max: usize) -> Self {
        self.config.max_point_clouds = max;
        self
    }

    /// Enable memory usage tracking
    pub fn with_memory_tracking(mut self) -> Self {
        self.config.memory_tracking = true;
        self
    }

    /// Build the debug configuration
    pub fn build(self) -> DebugConfig {
        self.config
    }
}

/// Callback trait for timing measurements
pub trait TimingCallback: Send + Sync {
    /// Called when a pipeline stage starts
    fn on_stage_start(&self, stage: &str, timestamp: Instant);

    /// Called when a pipeline stage ends
    fn on_stage_end(&self, stage: &str, duration: Duration, memory_usage: Option<usize>);
}

/// Callback trait for intermediate data output
pub trait DataCallback: Send + Sync {
    /// Called with intermediate data from pipeline stages
    fn on_intermediate_data(&self, stage: &str, data: &DebugData);

    /// Called with point cloud data from processing stages
    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud);
}

/// Callback trait for algorithm metrics and statistics
pub trait MetricsCallback: Send + Sync {
    /// Called with stage-specific metrics
    fn on_metrics(&self, stage: &str, metrics: &StageMetrics);

    /// Called with algorithm convergence and performance statistics
    fn on_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats);
}

/// Container for different types of debug data
#[derive(Debug, Clone)]
pub enum DebugData {
    /// Point cloud data with optional metadata
    PointCloud {
        cloud: PointCloud,
        metadata: HashMap<String, String>,
    },
    /// Board detection results with intermediate data
    DetectionResult {
        detections: Vec<BoardDetection>,
        confidence_scores: Vec<f64>,
        metadata: HashMap<String, String>,
    },
    /// Plane detection intermediate results
    PlaneData {
        planes: Vec<DetectedPlane>,
        inlier_counts: Vec<usize>,
        quality_scores: Vec<f64>,
        metadata: HashMap<String, String>,
    },
    /// Circle/hole detection results
    CircleData {
        holes: Vec<DetectedHole>,
        fitting_residuals: Vec<f64>,
        iteration_counts: Vec<usize>,
        metadata: HashMap<String, String>,
    },
    /// Generic key-value data
    Generic {
        data: HashMap<String, serde_json::Value>,
    },
}

/// Stage-specific performance and algorithmic metrics
#[derive(Debug, Clone)]
pub struct StageMetrics {
    /// Number of input points processed
    pub input_points: usize,
    /// Number of output points produced
    pub output_points: usize,
    /// Processing time for this stage
    pub processing_time: Duration,
    /// Memory usage during processing (if tracking enabled)
    pub memory_usage: Option<usize>,
    /// Stage-specific metrics
    pub custom_metrics: HashMap<String, f64>,
}

impl StageMetrics {
    /// Create new stage metrics
    pub fn new(input_points: usize, output_points: usize, processing_time: Duration) -> Self {
        Self {
            input_points,
            output_points,
            processing_time,
            memory_usage: None,
            custom_metrics: HashMap::new(),
        }
    }

    /// Add a custom metric
    pub fn add_metric<K: Into<String>>(&mut self, key: K, value: f64) {
        self.custom_metrics.insert(key.into(), value);
    }

    /// Set memory usage
    pub fn with_memory_usage(mut self, usage: usize) -> Self {
        self.memory_usage = Some(usage);
        self
    }
}

/// Algorithm convergence and performance statistics
#[derive(Debug, Clone)]
pub struct AlgorithmStats {
    /// Algorithm name
    pub algorithm: String,
    /// Number of iterations performed
    pub iterations: usize,
    /// Whether the algorithm converged
    pub converged: bool,
    /// Final residual/error value
    pub final_error: Option<f64>,
    /// Convergence tolerance used
    pub tolerance: Option<f64>,
    /// Custom algorithm-specific statistics
    pub custom_stats: HashMap<String, serde_json::Value>,
}

impl AlgorithmStats {
    /// Create new algorithm statistics
    pub fn new<S: Into<String>>(algorithm: S, iterations: usize, converged: bool) -> Self {
        Self {
            algorithm: algorithm.into(),
            iterations,
            converged,
            final_error: None,
            tolerance: None,
            custom_stats: HashMap::new(),
        }
    }

    /// Set final error value
    pub fn with_error(mut self, error: f64) -> Self {
        self.final_error = Some(error);
        self
    }

    /// Set convergence tolerance
    pub fn with_tolerance(mut self, tolerance: f64) -> Self {
        self.tolerance = Some(tolerance);
        self
    }

    /// Add custom statistic
    pub fn add_stat<K: Into<String>, V: Into<serde_json::Value>>(&mut self, key: K, value: V) {
        self.custom_stats.insert(key.into(), value.into());
    }
}

/// Debug context that threads through the processing pipeline
pub struct DebugContext {
    /// Debug configuration
    pub config: DebugConfig,
    /// Timing callback (if enabled)
    pub timing_callback: Option<Arc<dyn TimingCallback>>,
    /// Data callback (if enabled)
    pub data_callback: Option<Arc<dyn DataCallback>>,
    /// Metrics callback (if enabled)
    pub metrics_callback: Option<Arc<dyn MetricsCallback>>,
    /// Current stage start times (for timing measurements)
    stage_timers: HashMap<String, Instant>,
}

impl std::fmt::Debug for DebugContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DebugContext")
            .field("config", &self.config)
            .field(
                "timing_callback",
                &self
                    .timing_callback
                    .as_ref()
                    .map(|_| "Some(TimingCallback)"),
            )
            .field(
                "data_callback",
                &self.data_callback.as_ref().map(|_| "Some(DataCallback)"),
            )
            .field(
                "metrics_callback",
                &self
                    .metrics_callback
                    .as_ref()
                    .map(|_| "Some(MetricsCallback)"),
            )
            .field("stage_timers", &self.stage_timers)
            .finish()
    }
}

impl DebugContext {
    /// Create a new debug context with the given configuration
    pub fn new(config: DebugConfig) -> Self {
        Self {
            config,
            timing_callback: None,
            data_callback: None,
            metrics_callback: None,
            stage_timers: HashMap::new(),
        }
    }

    /// Set the timing callback
    pub fn with_timing_callback(mut self, callback: Arc<dyn TimingCallback>) -> Self {
        self.timing_callback = Some(callback);
        self
    }

    /// Set the data callback
    pub fn with_data_callback(mut self, callback: Arc<dyn DataCallback>) -> Self {
        self.data_callback = Some(callback);
        self
    }

    /// Set the metrics callback
    pub fn with_metrics_callback(mut self, callback: Arc<dyn MetricsCallback>) -> Self {
        self.metrics_callback = Some(callback);
        self
    }

    /// Check if debugging is enabled for the given stage
    pub fn is_stage_enabled(&self, stage: &str) -> bool {
        self.config.verbosity_level != DebugVerbosity::None
            && (self.config.output_stages.is_empty()
                || self.config.output_stages.contains(&stage.to_string()))
    }

    /// Start timing for a pipeline stage
    pub fn start_stage(&mut self, stage: &str) {
        if self.config.timing_enabled {
            let now = Instant::now();
            self.stage_timers.insert(stage.to_string(), now);

            if let Some(callback) = &self.timing_callback {
                callback.on_stage_start(stage, now);
            }
        }
    }

    /// End timing for a pipeline stage
    pub fn end_stage(&mut self, stage: &str) {
        if self.config.timing_enabled {
            if let Some(start_time) = self.stage_timers.remove(stage) {
                let duration = start_time.elapsed();
                let memory_usage = if self.config.memory_tracking {
                    // TODO: Implement actual memory usage tracking
                    Some(0)
                } else {
                    None
                };

                if let Some(callback) = &self.timing_callback {
                    callback.on_stage_end(stage, duration, memory_usage);
                }
            }
        }
    }

    /// Emit debug data for the given stage
    pub fn emit_data(&self, stage: &str, data: &DebugData) {
        if self.is_stage_enabled(stage) {
            if let Some(callback) = &self.data_callback {
                callback.on_intermediate_data(stage, data);
            }
        }
    }

    /// Emit point cloud data for the given stage
    pub fn emit_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        if self.is_stage_enabled(stage) {
            if let Some(callback) = &self.data_callback {
                callback.on_point_cloud(stage, cloud);
            }
        }
    }

    /// Emit metrics for the given stage
    pub fn emit_metrics(&self, stage: &str, metrics: &StageMetrics) {
        if self.is_stage_enabled(stage) {
            if let Some(callback) = &self.metrics_callback {
                callback.on_metrics(stage, metrics);
            }
        }
    }

    /// Emit algorithm statistics
    pub fn emit_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats) {
        if self.is_stage_enabled(stage) {
            if let Some(callback) = &self.metrics_callback {
                callback.on_algorithm_stats(stage, stats);
            }
        }
    }
}

/// No-op implementation of TimingCallback for zero overhead
#[derive(Debug)]
pub struct NoOpTimingCallback;

impl TimingCallback for NoOpTimingCallback {
    fn on_stage_start(&self, _stage: &str, _timestamp: Instant) {}
    fn on_stage_end(&self, _stage: &str, _duration: Duration, _memory_usage: Option<usize>) {}
}

/// No-op implementation of DataCallback for zero overhead
#[derive(Debug)]
pub struct NoOpDataCallback;

impl DataCallback for NoOpDataCallback {
    fn on_intermediate_data(&self, _stage: &str, _data: &DebugData) {}
    fn on_point_cloud(&self, _stage: &str, _cloud: &PointCloud) {}
}

/// No-op implementation of MetricsCallback for zero overhead
#[derive(Debug)]
pub struct NoOpMetricsCallback;

impl MetricsCallback for NoOpMetricsCallback {
    fn on_metrics(&self, _stage: &str, _metrics: &StageMetrics) {}
    fn on_algorithm_stats(&self, _stage: &str, _stats: &AlgorithmStats) {}
}

/// Console-based debug logger for development and testing
#[derive(Debug)]
pub struct ConsoleDebugLogger {
    pub verbose: bool,
}

impl ConsoleDebugLogger {
    pub fn new(verbose: bool) -> Self {
        Self { verbose }
    }
}

impl TimingCallback for ConsoleDebugLogger {
    fn on_stage_start(&self, stage: &str, timestamp: Instant) {
        if self.verbose {
            println!("TIMING: Starting stage '{}' at {:?}", stage, timestamp);
        }
    }

    fn on_stage_end(&self, stage: &str, duration: Duration, memory_usage: Option<usize>) {
        println!(
            "TIMING: Stage '{}' completed in {:.2}ms{}",
            stage,
            duration.as_secs_f64() * 1000.0,
            if let Some(mem) = memory_usage {
                format!(" ({}KB memory)", mem / 1024)
            } else {
                String::new()
            }
        );
    }
}

impl DataCallback for ConsoleDebugLogger {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        match data {
            DebugData::PointCloud { cloud, metadata } => {
                println!(
                    "DATA: Stage '{}' - PointCloud with {} points",
                    stage,
                    cloud.points.len()
                );
                if self.verbose {
                    for (key, value) in metadata {
                        println!("  {}: {}", key, value);
                    }
                }
            }
            DebugData::DetectionResult {
                detections,
                confidence_scores,
                metadata,
            } => {
                println!(
                    "DATA: Stage '{}' - {} detections with confidences: {:?}",
                    stage,
                    detections.len(),
                    confidence_scores
                );
                if self.verbose {
                    for (key, value) in metadata {
                        println!("  {}: {}", key, value);
                    }
                }
            }
            DebugData::PlaneData {
                planes,
                inlier_counts,
                quality_scores,
                metadata,
            } => {
                println!(
                    "DATA: Stage '{}' - {} planes with inliers: {:?}, quality: {:?}",
                    stage,
                    planes.len(),
                    inlier_counts,
                    quality_scores
                );
                if self.verbose {
                    for (key, value) in metadata {
                        println!("  {}: {}", key, value);
                    }
                }
            }
            DebugData::CircleData {
                holes,
                fitting_residuals,
                iteration_counts,
                metadata,
            } => {
                println!(
                    "DATA: Stage '{}' - {} holes with residuals: {:?}",
                    stage,
                    holes.len(),
                    fitting_residuals
                );
                if self.verbose {
                    for (key, value) in metadata {
                        println!("  {}: {}", key, value);
                    }
                }
            }
            DebugData::Generic { data } => {
                println!(
                    "DATA: Stage '{}' - Generic data with {} entries",
                    stage,
                    data.len()
                );
                if self.verbose {
                    for (key, value) in data {
                        println!("  {}: {}", key, value);
                    }
                }
            }
        }
    }

    fn on_point_cloud(&self, stage: &str, cloud: &PointCloud) {
        if self.verbose {
            println!(
                "CLOUD: Stage '{}' - {} points (frame: {})",
                stage,
                cloud.points.len(),
                cloud.frame_id
            );
        }
    }
}

impl MetricsCallback for ConsoleDebugLogger {
    fn on_metrics(&self, stage: &str, metrics: &StageMetrics) {
        println!(
            "METRICS: Stage '{}' - {}->{} points in {:.2}ms",
            stage,
            metrics.input_points,
            metrics.output_points,
            metrics.processing_time.as_secs_f64() * 1000.0
        );
        if self.verbose {
            for (key, value) in &metrics.custom_metrics {
                println!("  {}: {:.3}", key, value);
            }
        }
    }

    fn on_algorithm_stats(&self, stage: &str, stats: &AlgorithmStats) {
        println!(
            "ALGORITHM: Stage '{}' - {} ({} iterations, converged: {}, error: {:?})",
            stage, stats.algorithm, stats.iterations, stats.converged, stats.final_error
        );
        if self.verbose {
            for (key, value) in &stats.custom_stats {
                println!("  {}: {}", key, value);
            }
        }
    }
}

/// Pipeline stage names as constants for consistency
pub mod stages {
    pub const PLANE_DETECTION: &str = "plane_detection";
    pub const DIAMOND_FITTING: &str = "diamond_fitting";
    pub const HOLE_DETECTION: &str = "hole_detection";
    pub const BOARD_TRACKING: &str = "board_tracking";
    pub const ROI_MANAGEMENT: &str = "roi_management";
    pub const VALIDATION: &str = "validation";
    pub const PREPROCESSING: &str = "preprocessing";
}

/// Macros for conditional debug execution
#[macro_export]
macro_rules! debug_timing {
    ($ctx:expr, $stage:expr, $block:block) => {
        if $ctx.config.timing_enabled {
            $ctx.start_stage($stage);
            let result = $block;
            $ctx.end_stage($stage);
            result
        } else {
            $block
        }
    };
}

#[macro_export]
macro_rules! debug_emit_data {
    ($ctx:expr, $stage:expr, $data:expr) => {
        if $ctx.is_stage_enabled($stage) {
            $ctx.emit_data($stage, $data);
        }
    };
}

#[macro_export]
macro_rules! debug_emit_metrics {
    ($ctx:expr, $stage:expr, $metrics:expr) => {
        if $ctx.is_stage_enabled($stage) {
            $ctx.emit_metrics($stage, $metrics);
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Debug, Default)]
    struct TestTimingCallback {
        calls: Arc<Mutex<Vec<(String, String)>>>, // (stage, event_type)
    }

    impl TestTimingCallback {
        fn new() -> Self {
            Self::default()
        }

        fn get_calls(&self) -> Vec<(String, String)> {
            self.calls.lock().unwrap().clone()
        }
    }

    impl TimingCallback for TestTimingCallback {
        fn on_stage_start(&self, stage: &str, _timestamp: Instant) {
            self.calls
                .lock()
                .unwrap()
                .push((stage.to_string(), "start".to_string()));
        }

        fn on_stage_end(&self, stage: &str, _duration: Duration, _memory_usage: Option<usize>) {
            self.calls
                .lock()
                .unwrap()
                .push((stage.to_string(), "end".to_string()));
        }
    }

    #[test]
    fn test_debug_config_builder() {
        let config = DebugConfigBuilder::new()
            .with_timing()
            .capture_stage("test_stage")
            .verbosity(DebugVerbosity::Detailed)
            .max_point_clouds(20)
            .with_memory_tracking()
            .build();

        assert!(config.timing_enabled);
        assert_eq!(config.output_stages, vec!["test_stage"]);
        assert_eq!(config.verbosity_level, DebugVerbosity::Detailed);
        assert_eq!(config.max_point_clouds, 20);
        assert!(config.memory_tracking);
    }

    #[test]
    fn test_debug_context_timing() {
        let config = DebugConfigBuilder::new().with_timing().build();
        let callback = Arc::new(TestTimingCallback::new());
        let callback_clone = Arc::clone(&callback);

        let mut ctx = DebugContext::new(config).with_timing_callback(callback_clone);

        ctx.start_stage("test_stage");
        std::thread::sleep(Duration::from_millis(1));
        ctx.end_stage("test_stage");

        let calls = callback.get_calls();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0], ("test_stage".to_string(), "start".to_string()));
        assert_eq!(calls[1], ("test_stage".to_string(), "end".to_string()));
    }

    #[test]
    fn test_stage_metrics() {
        let mut metrics = StageMetrics::new(1000, 800, Duration::from_millis(100));
        metrics.add_metric("accuracy", 0.95);
        metrics = metrics.with_memory_usage(1024);

        assert_eq!(metrics.input_points, 1000);
        assert_eq!(metrics.output_points, 800);
        assert_eq!(metrics.processing_time, Duration::from_millis(100));
        assert_eq!(metrics.memory_usage, Some(1024));
        assert_eq!(metrics.custom_metrics.get("accuracy"), Some(&0.95));
    }

    #[test]
    fn test_algorithm_stats() {
        let mut stats = AlgorithmStats::new("RANSAC", 100, true)
            .with_error(0.001)
            .with_tolerance(0.01);
        stats.add_stat("inliers", 950);

        assert_eq!(stats.algorithm, "RANSAC");
        assert_eq!(stats.iterations, 100);
        assert!(stats.converged);
        assert_eq!(stats.final_error, Some(0.001));
        assert_eq!(stats.tolerance, Some(0.01));
    }
}
