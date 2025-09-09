# Contributing

Thank you for your interest in contributing to LCTK! This guide will help you get started.

## Getting Started

### Development Setup
1. Follow the [Build System](./build.md) guide to set up your environment
2. Fork the repository and create a feature branch
3. Make your changes following the coding standards below

### Coding Standards

#### Rust Code Style
- Use `cargo fmt` with nightly toolchain for consistent formatting
- Follow Rust naming conventions (snake_case for functions, PascalCase for types)
- Use named parameters in format strings: `println!("{error}")` instead of `println!("{}", error)`
- Avoid "Pokemon exception handling" - don't catch exceptions silently
- Prefer explicit error handling with `Result` types

#### Code Organization
- Initialize struct fields first, then construct the struct (avoid mutable structs)
- Use Arc for shared state in concurrent contexts
- Prefer move closures with locally cloned variables:
  ```rust
  let subscription = {
      let state = Arc::clone(&state);
      let publisher = Arc::clone(&publisher);

      node.create_subscription::<MessageType, _>(
          "topic_name",
          move |msg| callback(msg, &state, &publisher),
      )?
  };
  ```

## Development Process

### Branch Strategy
- `main`: Stable, production-ready code
- Feature branches: `feature/description` or `fix/description`
- Create pull requests against `main`

### Commit Guidelines
- Write clear, descriptive commit messages
- Use conventional commit format: `type(scope): description`
- Include tests for new functionality
- Update documentation as needed

### Testing Requirements
- All new code must include appropriate tests
- Ensure existing tests continue to pass
- Run the full test suite before submitting: `make test`
- Include performance benchmarks for algorithmic changes

## Pull Request Process

1. **Fork and Clone**: Fork the repository and clone your fork
2. **Create Branch**: Create a feature branch from main
3. **Implement Changes**: Make your changes following coding standards
4. **Test**: Run tests and ensure they pass
5. **Document**: Update documentation if needed
6. **Submit PR**: Create a pull request with clear description

### PR Requirements
- [ ] Code follows style guidelines
- [ ] Tests added for new functionality
- [ ] All tests pass
- [ ] Documentation updated
- [ ] No merge conflicts with main

## Areas for Contribution

### High Priority
- Performance optimization (GPU acceleration, algorithm improvements)
- Additional sensor support (thermal cameras, radar, IMU)
- Calibration accuracy improvements
- Documentation and examples

### Medium Priority
- UI/UX improvements for visualization
- Additional calibration algorithms
- Cross-platform compatibility
- Automated testing infrastructure

### Good First Issues
- Documentation improvements
- Code cleanup and refactoring
- Unit test additions
- Bug fixes and error handling

## Community Guidelines

### Communication
- Be respectful and inclusive
- Ask questions in issues or discussions
- Provide constructive feedback in code reviews
- Help others learn and contribute

### Code of Conduct
We follow a standard code of conduct:
- Be welcoming and inclusive
- Respect different viewpoints and experiences
- Accept constructive criticism gracefully
- Focus on what's best for the community

## Getting Help

### Documentation
- Check the [Build System](./build.md) guide for setup issues
- Review [Architecture](../architecture/overview.md) for design questions
- See [Testing](./testing.md) for test-related guidance

### Communication Channels
- GitHub Issues: Bug reports and feature requests
- GitHub Discussions: General questions and ideas
- Pull Request Comments: Code-specific discussions

Thank you for contributing to LCTK!