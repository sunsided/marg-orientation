use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use core::ops::Mul;

#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct AccelerometerReading<T> {
    /// The acceleration along the x-axis, in meters per second.
    pub x: T,
    /// The acceleration along the y-axis, in meters per second.
    pub y: T,
    /// The acceleration along the z-axis, in meters per second.
    pub z: T,
}

impl<T> AccelerometerReading<T> {
    /// Initializes a new [`AccelerometerReading`] instance.
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    /// Constructs a new [`AccelerometerReading`] instance from a reading in a given coordinate frame.
    #[cfg(feature = "coordinate-frame")]
    #[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
    pub fn from_ned<C>(coordinate: C) -> Self
    where
        C: Into<coordinate_frame::NorthEastDown<T>>,
        T: Clone,
    {
        let coordinate = coordinate.into();
        Self {
            x: coordinate.x(),
            y: coordinate.y(),
            z: coordinate.z(),
        }
    }

    /// Returns the length of the [`AccelerometerReading`] vector.
    #[inline(always)]
    pub const fn len(&self) -> usize {
        3
    }
}

impl<T> Default for AccelerometerReading<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for AccelerometerReading<T>
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

impl<T> Debug for AccelerometerReading<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("AccelerometerReading")
            .field(&self.x)
            .field(&self.y)
            .field(&self.z)
            .finish()
    }
}

impl<T> Mul<T> for AccelerometerReading<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = AccelerometerReading<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs.clone(),
            z: self.z * rhs.clone(),
        }
    }
}

#[cfg(feature = "coordinate-frame")]
#[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
impl<T, C> From<C> for AccelerometerReading<T>
where
    C: coordinate_frame::CoordinateFrame<Type = T>,
    T: Copy + coordinate_frame::SaturatingNeg<Output = T>,
{
    fn from(value: C) -> Self {
        Self::from_ned(value.to_ned())
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::Index<usize> for AccelerometerReading<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of bounds"),
        }
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::IndexMut<usize> for AccelerometerReading<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl_standard_traits!(AccelerometerReading, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = AccelerometerReading::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = AccelerometerReading::<f32> {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
