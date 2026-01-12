# Contributing

Thank you for contributing to LCTK! This guide covers coding standards, workflow, and pull request process.

## Quick Start

```bash
# Fork and clone
git clone https://github.com/your-username/LCTK.git
cd LCTK

# Create branch
git checkout -b feature/my-feature

# Setup environment
./setup-dev-env.sh -y
just build

# Make changes, test, commit
cargo test --workspace
git add .
git commit -m "feat: add new feature"

# Push and create PR
git push origin feature/my-feature
```

## Coding Standards

### Rust Style

**Use rustfmt:**
```bash
cargo fmt --all
```

**Use clippy:**
```bash
cargo clippy --all-targets --all-features
```

**Naming conventions:**
- Functions/variables: `snake_case`
- Types/traits: `PascalCase`
- Constants: `SCREAMING_SNAKE_CASE`

### Code Patterns

**Named parameters in format strings:**
```rust
// GOOD
println!("{error}");

// BAD
println!("{}", error);
```

**Functional struct initialization:**
```rust
// GOOD
let state = State {
    field1: value1,
    field2: value2,
};

// BAD (avoid mutable structs)
let mut state = State::new();
state.field1 = value1;
state.field2 = value2;
```

**Closure variable cloning:**
```rust
// GOOD
let subscription = {
    let state = Arc::clone(&state);
    node.create_subscription::<Message, _>(
        "topic",
        move |msg| callback(msg, &state),
    )?
};

// BAD (clutters namespace)
let state_clone = Arc::clone(&state);
let subscription = node.create_subscription::<Message, _>(
    "topic",
    move |msg| callback(msg, &state_clone),
)?;
```

**Explicit error handling:**
```rust
// GOOD
let result = operation()?;
// or
let result = operation().context("Failed to perform operation")?;

// BAD (silent errors)
let _ = operation(); // Don't do this!
```

### Documentation

**Add rustdoc comments:**
```rust
/// Detects ArUco markers in an image.
///
/// # Arguments
/// * `image` - Input image as OpenCV Mat
///
/// # Returns
/// Vector of detected markers with corners and IDs
pub fn detect(&self, image: &Mat) -> Result<Vec<Detection>> {
    // ...
}
```

## Development Workflow

### 1. Branch Strategy

- `main`: Stable, production code
- `feature/description`: New features
- `fix/description`: Bug fixes

### 2. Commit Messages

Use conventional commits:

```
feat: add board detection debug mode
fix: correct bounding box coordinate calculation
docs: update calibration workflow guide
test: add unit tests for plane estimator
refactor: simplify ArUco detection pipeline
```

### 3. Testing

**Before committing:**
```bash
# Format
just format

# Lint
just lint

# Test
just test

# Build
just build
```

### 4. Pull Request

**Checklist:**
- [ ] Code formatted (`cargo fmt`)
- [ ] No clippy warnings
- [ ] Tests added for new features
- [ ] All tests pass
- [ ] Documentation updated
- [ ] CLAUDE.md updated (if adding known issues/patterns)

**PR Template:**
```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
How was this tested?

## Related Issues
Closes #123
```

## Common Contribution Areas

### Good First Issues

- Add unit tests for existing code
- Improve documentation
- Fix typos and formatting
- Add code examples

### High Priority

- Performance optimization
- Bug fixes in calibration algorithms
- ROS 2 node stability improvements
- New sensor support

### Medium Priority

- Additional calibration methods
- Visualization enhancements
- Configuration improvements
- Build system optimization

## Building ROS Packages

**Use justfile for building:**

```bash
# Build all packages
just build

# Clean and rebuild
just clean && just build
```

See [Build System](./build-system.md) for details.

## Code Review Process

### What Reviewers Look For

1. **Correctness:** Code works as intended
2. **Tests:** Adequate test coverage
3. **Style:** Follows coding standards
4. **Documentation:** Public APIs documented
5. **Performance:** No obvious inefficiencies

### Responding to Reviews

- Address all comments
- Ask questions if unclear
- Make requested changes
- Mark conversations resolved
- Thank reviewers!

## Development Tips

### Debugging

```bash
# Enable debug logging
export RUST_LOG=debug

# Enable ROS logging
export RCUTILS_LOGGING_LEVEL=DEBUG

# Run with debugger
rust-gdb target/debug/my_node
```

### Performance Profiling

```bash
# CPU profiling
perf record -g target/release/my_node
perf report

# Memory profiling
valgrind target/release/my_node
```

### IDE Setup

**VS Code:**
- Install `rust-analyzer` extension
- Install `ROS` extension
- Use workspace settings from `.vscode/settings.json`

**CLion:**
- Open project root
- Auto-detects CMake + Cargo
- Use Rust plugin

## Community Guidelines

### Be Respectful

- Welcome newcomers
- Be patient with questions
- Provide constructive feedback
- Celebrate contributions

### Communication

- **Issues:** Bug reports, feature requests
- **Discussions:** Questions, ideas, help
- **PRs:** Code-specific discussion

## Getting Help

**Build issues:** See [Build System](./build-system.md)

**Testing questions:** See [Testing](./testing.md)

**Architecture questions:** See [Architecture](./architecture.md)

**Stuck?** Ask in GitHub Discussions!

## License

By contributing, you agree that your contributions will be licensed under the same license as the project.

---

Thank you for making LCTK better! 🚀
