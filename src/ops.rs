#[cfg(feature = "uom")]
use uom::marker::Div;
#[cfg(feature = "uom")]
use uom::si::{Dimension, Quantity, Units, ISQ};
#[cfg(feature = "uom")]
use uom::typenum::{Integer, PartialDiv, PartialQuot, P2};

/// Trait for types implementing the square root operation.
pub trait Sqrt {
    /// Output type.
    type Output;
    /// Calculate square root of this value.
    fn sqrt(self) -> Self::Output;
}

impl Sqrt for f32 {
    type Output = f32;
    fn sqrt(self) -> Self::Output {
        f32::sqrt(self)
    }
}

impl Sqrt for f64 {
    type Output = f64;
    fn sqrt(self) -> Self::Output {
        f64::sqrt(self)
    }
}

pub trait Norm {
    type Output;
    fn norm(self) -> Self::Output;
}

#[cfg(feature = "uom")]
impl<D, U> Sqrt for Quantity<D, U, f64>
where
    D: Dimension + ?Sized,
    U: Units<f64> + ?Sized,
    D::L: PartialDiv<P2>,
    <D::L as PartialDiv<P2>>::Output: Integer,
    D::M: PartialDiv<P2>,
    <D::M as PartialDiv<P2>>::Output: Integer,
    D::T: PartialDiv<P2>,
    <D::T as PartialDiv<P2>>::Output: Integer,
    D::I: PartialDiv<P2>,
    <D::I as PartialDiv<P2>>::Output: Integer,
    D::Th: PartialDiv<P2>,
    <D::Th as PartialDiv<P2>>::Output: Integer,
    D::N: PartialDiv<P2>,
    <D::N as PartialDiv<P2>>::Output: Integer,
    D::J: PartialDiv<P2>,
    <D::J as PartialDiv<P2>>::Output: Integer,
    D::Kind: Div,
{
    type Output = Quantity<
        ISQ<
            PartialQuot<D::L, P2>,
            PartialQuot<D::M, P2>,
            PartialQuot<D::T, P2>,
            PartialQuot<D::I, P2>,
            PartialQuot<D::Th, P2>,
            PartialQuot<D::N, P2>,
            PartialQuot<D::J, P2>,
        >,
        U,
        f64,
    >;
    fn sqrt(self) -> Self::Output {
        Self::sqrt(self)
    }
}

#[cfg(test)]
mod test {
    use super::Sqrt;

    #[test]
    fn no_recursion_during_sqrt() {
        Sqrt::sqrt(4.0);
    }
}
