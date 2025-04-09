# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build/Test Commands
- `cargo build` - Build the library
- `cargo test` - Run all tests
- `cargo test test_name` - Run a specific test
- `cargo run --example linux_basic` - Run basic Linux example
- `cargo run --example linux_advanced` - Run advanced Linux example
- `cargo run --example linux_realtime_plot --features="plotting"` - Run plotting example
- `cargo clippy` - Run linter

## Code Style Guidelines
- Use 4-space indentation and rustfmt formatting
- Structs/enums use PascalCase, functions/variables use snake_case
- Group imports: external crates first, then internal modules
- Error handling: use custom `Icm20948Error` enum and Result type
- Document public API with /// comments and modules with //! comments
- Separate concerns into dedicated modules/files
- Use Rust's type system for safety and clear abstraction boundaries
- Return Result<T, Icm20948Error> for functions that can fail
- Use ? operator for error propagation
- Directories Arduino_ICM20948_DMP_Full-Function and SparkFun_ICM-20948_ArduinoLibrary contains a C implementation of working library