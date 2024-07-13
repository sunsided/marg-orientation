use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use core::ops::Mul;
use uniform_array_derive::UniformArray;

#[derive(UniformArray)]
#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct MagnetometerNoise<T> {
    /// The noise of the magnetic field strength along the x-axis, in Gauss.
    pub x: T,
    /// The noise of the magnetic field strength along the y-axis, in Gauss.
    pub y: T,
    /// The noise of the magnetic field strength along the z-axis, in Gauss.
    pub z: T,
}

impl<T> MagnetometerNoise<T> {
    /// Initializes a new [`MagnetometerNoise`] instance.
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl<T> Default for MagnetometerNoise<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for MagnetometerNoise<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            x: self.x.clone(),
            y: self.y.clone(),
            z: self.z.clone(),
        }
    }
}

impl<T> Debug for MagnetometerNoise<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("MagnetometerReading")
            .field(&self.x)
            .field(&self.y)
            .field(&self.z)
            .finish()
    }
}

impl<T> Mul<T> for MagnetometerNoise<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = MagnetometerNoise<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs.clone(),
            z: self.z * rhs.clone(),
        }
    }
}

impl_standard_traits!(MagnetometerNoise, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = MagnetometerNoise::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = MagnetometerNoise::<f32> {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
