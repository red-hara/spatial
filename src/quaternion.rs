use crate::vector::Vector;
use core::ops::{Add, Mul, Neg, Sub};
#[cfg(feature = "num")]
use num::{Float, One, Zero};

#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Quaternion<T> {
    w: T,
    i: T,
    j: T,
    k: T,
}

#[cfg(feature = "num")]
impl<T> Quaternion<T>
where
    T: One + Zero,
{
    pub fn identity() -> Self {
        let w = T::one();
        let i = T::zero();
        let j = T::zero();
        let k = T::zero();
        Self { w, i, j, k }
    }
}

#[cfg(feature = "num")]
impl<T> Quaternion<T>
where
    T: Float,
{
    pub fn from_angle_axis(angle: T, axis: Vector<T>) -> Self {
        if let Some(normalized) = axis.normalized_checked() {
            let angle = angle / (T::one() + T::one());
            let cos = angle.cos();
            let sin = angle.sin();
            let i = normalized.x() * sin;
            let j = normalized.y() * sin;
            let k = normalized.z() * sin;
            let w = cos;
            Self { w, i, j, k }
        } else {
            Self::identity()
        }
    }

    pub fn into_angle_axis(self) -> (T, Vector<T>) {
        let half_angle = self.w.max(-T::one()).min(T::one()).acos();
        if half_angle < T::epsilon() {
            (T::zero(), Vector::zero())
        } else {
            let sin = half_angle.sin();
            let axis = Vector::new(self.i, self.j, self.k) / sin;
            (half_angle * (T::one() + T::one()), axis)
        }
    }

    pub fn slerp(self, other: Self, progress: T) -> Self {
        let dot = self.dot(other);
        let (other, dot) = if dot < T::zero() {
            (-other, -dot)
        } else {
            (other, dot)
        };
        if dot >= T::one() {
            return self;
        }
        let dot = dot.min(T::one());
        let omega = dot.acos();
        let sin_omega = omega.sin();
        let a = ((T::one() - progress) * omega).sin() / sin_omega;
        let b = (progress * omega).sin() / sin_omega;
        self * a + other * b
    }
}

impl<T> Quaternion<T>
where
    T: Mul<Output = T> + Add<Output = T> + Sub<Output = T> + Copy,
{
    pub fn multiply(self, other: Self) -> Self {
        let w = self.w * other.w - self.i * other.i - self.j * other.j - self.k * other.k;
        let i = self.w * other.i + self.i * other.w + self.j * other.k - self.k * other.j;
        let j = self.w * other.j - self.i * other.k + self.j * other.w + self.k * other.i;
        let k = self.w * other.k + self.i * other.j - self.j * other.i + self.k * other.w;
        Self { w, i, j, k }
    }

    pub fn dot(self, other: Self) -> T {
        self.w * other.w + self.i * other.i + self.j * other.j + self.k * other.k
    }
}

impl<T> Quaternion<T> {
    pub fn rotate<U, R>(self, vector: Vector<U>) -> Vector<R>
    where
        T: Copy
            + Mul<U, Output = R>
            + Mul<R, Output = R>
            + Mul<Output = T>
            + Add<Output = T>
            + Sub<Output = T>,
        U: Copy,
        R: Copy + Add<Output = R> + Sub<Output = R>,
    {
        let (w, i, j, k) = (self.w, self.i, self.j, self.k);
        let (x, y, z) = (vector.x(), vector.y(), vector.z());
        let prep_x = i * j * y + i * k * z + j * w * z - k * w * y;
        let result_x = prep_x + prep_x + (i * i - j * j - k * k + w * w) * x;
        let prep_y = i * j * x - i * w * z + j * k * z + k * w * x;
        let result_y = prep_y + prep_y + (j * j - i * i - k * k + w * w) * y;
        let prep_z = i * k * x + i * w * y + j * k * y - j * w * x;
        let result_z = prep_z + prep_z + (w * w - i * i - j * j + k * k) * z;
        Vector::new(result_x, result_y, result_z)
    }

    pub fn inverse(self) -> Self
    where
        T: Neg<Output = T>,
    {
        Self {
            w: self.w,
            i: -self.i,
            j: -self.j,
            k: -self.k,
        }
    }
}

impl<T> Add for Quaternion<T>
where
    T: Add<Output = T>,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            w: self.w + other.w,
            i: self.i + other.i,
            j: self.j + other.j,
            k: self.k + other.k,
        }
    }
}

impl<T> Neg for Quaternion<T>
where
    T: Neg<Output = T>,
{
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            w: -self.w,
            i: -self.i,
            j: -self.j,
            k: -self.k,
        }
    }
}

impl<T> Mul<Quaternion<T>> for Quaternion<T>
where
    T: Mul<Output = T> + Add<Output = T> + Sub<Output = T> + Copy,
{
    type Output = Self;
    fn mul(self, other: Self) -> Self::Output {
        self.multiply(other)
    }
}

impl<T> Mul<T> for Quaternion<T>
where
    T: Mul<Output = T> + Copy,
{
    type Output = Self;
    fn mul(self, other: T) -> Self::Output {
        Quaternion {
            w: self.w * other,
            i: self.i * other,
            j: self.j * other,
            k: self.k * other,
        }
    }
}
