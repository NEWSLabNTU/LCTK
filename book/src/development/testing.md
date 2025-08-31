# Testing

LCTK employs a comprehensive testing strategy covering unit tests, integration tests, and performance validation.

## Testing Strategy

### Unit Tests
- **Core algorithms**: Individual function and method testing
- **Data structures**: Validation of serialization/deserialization  
- **Mathematical operations**: Precision and correctness verification
- **Error handling**: Edge case and failure mode testing

### Integration Tests  
- **ROS 2 node functionality**: End-to-end message flow validation
- **Calibration pipelines**: Complete workflow testing
- **Cross-component communication**: Interface contract verification
- **Configuration handling**: Parameter and launch file testing

### Performance Tests
- **Benchmarking**: Speed and memory usage measurement
- **Scalability**: Multi-sensor and distributed processing validation
- **Real-time constraints**: Latency and throughput verification
- **Resource utilization**: CPU, memory, and GPU efficiency analysis

## Test Organization

### Rust Testing
```bash
# Run all unit tests
cargo test

# Run specific test module
cargo test --bin aruco_locator_node

# Run with output
cargo test -- --nocapture

# Performance benchmarks
cargo bench
```

### ROS 2 Testing
```bash  
# Integration tests
colcon test --packages-select <package>

# Test results  
colcon test-result --verbose
```

### Calibration Accuracy Tests
```bash
# Run calibration validation
make test_calibration

# Accuracy benchmarks
./scripts/validate_accuracy.py
```

## Test Data

### Synthetic Datasets
- Generated ArUco patterns
- Simulated point clouds
- Known ground truth transformations
- Controlled noise and occlusion scenarios

### Real-world Datasets  
- Multi-environment calibration data
- Various sensor configurations
- Different lighting and weather conditions
- Edge cases and failure scenarios

## Continuous Integration

The project uses automated testing for:
- Pull request validation
- Performance regression detection  
- Cross-platform compatibility
- Documentation generation

## Quality Metrics

### Code Coverage
- Target: >80% test coverage
- Critical paths: >95% coverage
- Regular coverage reporting

### Performance Benchmarks
- Baseline performance tracking
- Regression detection
- Optimization validation