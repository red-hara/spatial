#[cfg(feature = "num")]
use num::Float;

/// Trait for types implementing the square root operation.
pub trait Sqrt {
    /// Output type.
    type Output;
    /// Calculate square root of this value.
    fn sqrt(self) -> Self::Output;
}

#[cfg(feature = "num")]
impl<T> Sqrt for T
where
    T: Float,
{
    type Output = T;
    fn sqrt(self) -> Self::Output {
        Float::sqrt(self)
    }
}
