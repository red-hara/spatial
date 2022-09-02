use crate::ops::Sqrt;
use core::ops::{Add, Div, Mul, Neg, Sub};
#[cfg(feature = "num")]
use num::{Float, One, Zero};

#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Vector<T> {
    x: T,
    y: T,
    z: T,
}

impl<T> Vector<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    pub fn map<F: Fn(T) -> R, R>(self, mapper: F) -> Vector<R> {
        Vector {
            x: mapper(self.x),
            y: mapper(self.y),
            z: mapper(self.z),
        }
    }
}

#[cfg(feature = "num")]
impl<T> Vector<T>
where
    T: Zero,
{
    pub fn zero() -> Self {
        let x = T::zero();
        let y = T::zero();
        let z = T::zero();
        Self { x, y, z }
    }
}
#[cfg(feature = "num")]
impl<T> Vector<T>
where
    T: One + Zero,
{
    pub fn unit_x() -> Self {
        let x = T::one();
        let y = T::zero();
        let z = T::zero();
        Self { x, y, z }
    }

    pub fn unit_y() -> Self {
        let x = T::zero();
        let y = T::one();
        let z = T::zero();
        Self { x, y, z }
    }

    pub fn unit_z() -> Self {
        let x = T::zero();
        let y = T::zero();
        let z = T::one();
        Self { x, y, z }
    }
}

impl<T> Vector<T> {
    pub fn x_ref(&self) -> &T {
        &self.x
    }

    pub fn x_mut(&mut self) -> &mut T {
        &mut self.x
    }

    pub fn set_x(&mut self, x: T) {
        self.x = x;
    }

    pub fn y_ref(&self) -> &T {
        &self.y
    }

    pub fn y_mut(&mut self) -> &mut T {
        &mut self.y
    }

    pub fn set_y(&mut self, y: T) {
        self.y = y;
    }

    pub fn z_ref(&self) -> &T {
        &self.z
    }

    pub fn z_mut(&mut self) -> &mut T {
        &mut self.z
    }

    pub fn set_z(&mut self, z: T) {
        self.z = z;
    }
}

impl<T> Vector<T>
where
    T: Copy,
{
    pub fn x(&self) -> T {
        self.x
    }

    pub fn y(&self) -> T {
        self.y
    }

    pub fn z(&self) -> T {
        self.z
    }
}

impl<T> Vector<T> {
    pub fn norm<I>(self) -> T
    where
        T: Copy + Mul<Output = I>,
        I: Sqrt<Output = T> + Add<Output = I>,
    {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalized_unchecked<I, R>(self) -> Vector<R>
    where
        T: Copy + Mul<Output = I> + Div<T, Output = R>,
        I: Sqrt<Output = T> + Add<Output = I>,
    {
        self / self.norm()
    }

    pub fn cross(self, other: Self) -> Self
    where
        T: Copy + Mul<Output = T> + Sub<Output = T>,
    {
        Vector {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
}

#[cfg(feature = "num")]
impl<T> Vector<T>
where
    T: Float + Div<Output = T>,
{
    pub fn normalized_checked(self) -> Option<Self> {
        let norm = self.norm();
        if norm < T::epsilon() {
            None
        } else {
            Some(self / norm)
        }
    }
}

impl<T> Vector<T> {
    pub fn interpolate<U>(self, other: Self, progress: U) -> Self
    where
        T: Copy + Mul<U, Output = T> + Add<Output = T> + Sub<Output = T>,
        U: Copy,
    {
        self + (other - self) * progress
    }

    pub fn dot<U, R>(self, other: Vector<U>) -> R
    where
        T: Mul<U, Output = R>,
        R: Add<Output = R>,
    {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn project_on<U>(self, other: Vector<U>) -> Self
    where
        T: Copy + Add<Output = T> + Mul<U, Output = T> + Div<U, Output = T>,
        U: Copy + Add<Output = U> + Mul<Output = U> + Mul<T, Output = T>,
    {
        other * (self.dot(other) / other.dot(other))
    }

    pub fn reject_from<U>(self, other: Vector<U>) -> Self
    where
        T: Copy + Add<Output = T> + Sub<Output = T> + Mul<U, Output = T> + Div<U, Output = T>,
        U: Copy + Add<Output = U> + Mul<Output = U> + Mul<T, Output = T>,
    {
        let projection = self.project_on(other);
        self - projection
    }
}

impl<T, U, R> Div<U> for Vector<T>
where
    T: Div<U, Output = R>,
    U: Copy,
{
    type Output = Vector<R>;
    fn div(self, scalar: U) -> Self::Output {
        let x = self.x / scalar;
        let y = self.y / scalar;
        let z = self.z / scalar;
        Self::Output { x, y, z }
    }
}

impl<T, U, R> Mul<U> for Vector<T>
where
    T: Mul<U, Output = R>,
    U: Copy,
{
    type Output = Vector<R>;
    fn mul(self, scalar: U) -> Self::Output {
        let x = self.x * scalar;
        let y = self.y * scalar;
        let z = self.z * scalar;
        Self::Output { x, y, z }
    }
}

impl<T> Add for Vector<T>
where
    T: Add<Output = T>,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl<T> Sub for Vector<T>
where
    T: Sub<Output = T>,
{
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl<T> Neg for Vector<T>
where
    T: Neg<Output = T>,
{
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}
