#![no_std]

//! Spatial representation based on the vector-quaternion pairs.

/// Spatial pose, vector-quaternion pair.
pub mod pose;
/// Spatial rotation.
pub mod quaternion;
/// Spatial translation.
pub mod vector;

/// Helper math operations.
pub mod ops;
