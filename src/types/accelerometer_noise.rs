use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use core::ops::Mul;
use uniform_array_derive::UniformArray;

#[derive(UniformArray)]
#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct AccelerometerNoise<T> {
    /// The acceleration noise along the x-axis, in meters per second.
    pub x: T,
    /// The acceleration noise along the y-axis, in meters per second.
    pub y: T,
    /// The acceleration noise along the z-axis, in meters per second.
    pub z: T,
}

impl<T> AccelerometerNoise<T> {
    /// Initializes a new [`AccelerometerNoise`] instance.
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl<T> Default for AccelerometerNoise<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for AccelerometerNoise<T>
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

impl<T> Debug for AccelerometerNoise<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("AccelerometerNoise")
            .field(&self.x)
            .field(&self.y)
            .field(&self.z)
            .finish()
    }
}

impl<T> Mul<T> for AccelerometerNoise<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = AccelerometerNoise<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs.clone(),
            z: self.z * rhs.clone(),
        }
    }
}

impl_standard_traits!(AccelerometerNoise, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = AccelerometerNoise::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = AccelerometerNoise::<f32> {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
