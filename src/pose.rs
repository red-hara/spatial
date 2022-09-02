use crate::quaternion::Quaternion;
use crate::vector::Vector;
use core::ops::{Add, Mul, Neg, Sub};
#[cfg(feature = "num")]
use num::{One, Zero};

/// Spatial pose in 3D space.
/// Consists of consecutive translation and rotation in parent space.
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Pose<T, R> {
    translation: Vector<T>,
    rotation: Quaternion<R>,
}

impl<T, R> Pose<T, R> {
    /// Create new `Pose` with given translation and rotation.
    pub fn new(translation: Vector<T>, rotation: Quaternion<R>) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    /// Create pose representing no translation and no rotation.
    #[cfg(feature = "num")]
    pub fn identity() -> Self
    where
        T: Zero,
        R: One + Zero,
    {
        let translation = Vector::zero();
        let rotation = Quaternion::identity();
        Self {
            translation,
            rotation,
        }
    }

    /// Get reference to stored translation.
    pub fn translation_ref(&self) -> &Vector<T> {
        &self.translation
    }

    /// Get reference to stored rotation.
    pub fn rotation_ref(&self) -> &Quaternion<R> {
        &self.rotation
    }

    /// Get mutable reference to stored translation.
    pub fn translation_mut(&mut self) -> &mut Vector<T> {
        &mut self.translation
    }

    /// Get mutable reference to stored rotation.
    pub fn rotation_mut(&mut self) -> &mut Quaternion<R> {
        &mut self.rotation
    }

    /// Set translation.
    pub fn set_translation(&mut self, translation: Vector<T>) {
        self.translation = translation;
    }

    /// Set rotation.
    pub fn set_rotation(&mut self, rotation: Quaternion<R>) {
        self.rotation = rotation;
    }

    /// Get translation.
    pub fn translation(&self) -> Vector<T>
    where
        T: Copy,
    {
        self.translation
    }

    /// Get rotation.
    pub fn rotation(&self) -> Quaternion<R>
    where
        R: Copy,
    {
        self.rotation
    }
}

impl<T, R> Pose<T, R>
where
    T: Copy + Add<Output = T> + Sub<Output = T>,
    R: Copy + Mul<Output = R> + Add<Output = R> + Sub<Output = R> + Mul<T, Output = T>,
{
    /// Calculate new pose based on consecutive application of this and other poses.
    /// ```
    /// # use spatial::vector::Vector;
    /// # use spatial::quaternion::Quaternion;
    /// # use spatial::pose::Pose;
    /// # use core::f64::consts::{FRAC_PI_2, FRAC_PI_4};
    /// let first = Pose::new(
    ///     Vector::new(0.0, 0.0, 2.0),
    ///     Quaternion::from_angle_axis(FRAC_PI_2, Vector::unit_z()),
    /// );
    /// let second = Pose::new(
    ///     Vector::new(1.0, 0.0, 0.0),
    ///     Quaternion::from_angle_axis(FRAC_PI_4, Vector::unit_x()),
    /// );
    /// let result = first.combine(second);
    /// let expected = Pose::new(
    ///     Vector::new(0.0, 1.0, 2.0),
    ///     Quaternion::from_angle_axis(FRAC_PI_2, Vector::unit_z()) *
    ///         Quaternion::from_angle_axis(FRAC_PI_4, Vector::unit_x()),
    /// );
    /// assert!((result.translation() - expected.translation()).norm() < 1e-3);
    /// assert!((result.rotation().dot(expected.rotation()) - 1.0).abs() < 1e-3);
    /// ```
    pub fn combine(self, other: Self) -> Self {
        let translation = self.translation + self.rotation.rotate(other.translation);
        let rotation = self.rotation * other.rotation;
        Self {
            translation,
            rotation,
        }
    }

    /// Apply transform stored in this pose to given vector.
    /// ```
    /// # use spatial::vector::Vector;
    /// # use spatial::quaternion::Quaternion;
    /// # use spatial::pose::Pose;
    /// # use core::f64::consts::FRAC_PI_2;
    /// let pose = Pose::new(
    ///     Vector::new(1.0, 2.0, 3.0),
    ///     Quaternion::from_angle_axis(FRAC_PI_2, Vector::unit_z()),
    /// );
    /// let vector = Vector::unit_x();
    /// let result = pose.apply_to(vector);
    /// let expected = Vector::new(1.0, 3.0, 3.0);
    /// assert!((result - expected).norm() < 1e-3);
    /// ```
    pub fn apply_to(self, other: Vector<T>) -> Vector<T> {
        self.translation + self.rotation.rotate(other)
    }

    /// Calculates inverse translation such that `a * a.inverse() = Pose::identity()`.
    /// ```
    /// # use spatial::vector::Vector;
    /// # use spatial::quaternion::Quaternion;
    /// # use spatial::pose::Pose;
    /// # use core::f64::consts::FRAC_PI_4;
    /// let pose = Pose::new(
    ///     Vector::new(1.0, 2.0, 3.0),
    ///     Quaternion::from_angle_axis(FRAC_PI_4, Vector::unit_x()),
    /// );
    /// let result = pose * pose.inverse();
    /// let expected = Pose::identity();
    /// assert!((result.translation() - expected.translation()).norm() < 1e-3);
    /// assert!((result.rotation().dot(expected.rotation()) - 1.0).abs() < 1e-3);
    /// ```
    pub fn inverse(self) -> Self
    where
        T: Neg<Output = T>,
        R: Neg<Output = R>,
    {
        let inverse_rotation = self.rotation.inverse();
        Self {
            translation: inverse_rotation.rotate(-self.translation),
            rotation: inverse_rotation,
        }
    }
}

/// Calculate result of pose combination operation.
impl<T, R> Mul for Pose<T, R>
where
    T: Copy + Add<Output = T> + Sub<Output = T>,
    R: Copy + Mul<Output = R> + Add<Output = R> + Sub<Output = R> + Mul<T, Output = T>,
{
    type Output = Self;
    fn mul(self, other: Self) -> Self::Output {
        self.combine(other)
    }
}

/// Apply transform stored in this pose to given vector.
impl<T, R> Mul<Vector<T>> for Pose<T, R>
where
    T: Copy + Add<Output = T> + Sub<Output = T>,
    R: Copy + Mul<Output = R> + Add<Output = R> + Sub<Output = R> + Mul<T, Output = T>,
{
    type Output = Vector<T>;
    fn mul(self, other: Vector<T>) -> Self::Output {
        self.apply_to(other)
    }
}
