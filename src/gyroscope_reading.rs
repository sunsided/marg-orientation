use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use core::ops::{Mul, Sub};

#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct GyroscopeReading<T> {
    /// The angular rate around the x-axis, in radians per second.
    pub omega_x: T,
    /// The angular rate around the y-axis, in radians per second.
    pub omega_y: T,
    /// The angular rate around the z-axis, in radians per second.
    pub omega_z: T,
}

impl<T> GyroscopeReading<T> {
    /// Initializes a new [`GyroscopeReading`] instance.
    #[inline(always)]
    pub const fn new(omega_x: T, omega_y: T, omega_z: T) -> Self {
        Self {
            omega_x,
            omega_y,
            omega_z,
        }
    }

    /// Constructs a new [`GyroscopeReading`] instance from a reading in a given coordinate frame.
    #[cfg(feature = "coordinate-frame")]
    #[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
    pub fn north_east_down<C>(coordinate: C) -> Self
    where
        C: Into<coordinate_frame::NorthEastDown<T>>,
        T: Clone,
    {
        let coordinate = coordinate.into();
        Self {
            omega_x: coordinate.x(),
            omega_y: coordinate.y(),
            omega_z: coordinate.z(),
        }
    }

    /// Returns the length of the [`GyroscopeReading`] vector.
    #[inline(always)]
    #[allow(unused)]
    pub const fn len(&self) -> usize {
        3
    }
}

impl<T> Default for GyroscopeReading<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for GyroscopeReading<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            omega_x: self.omega_x.clone(),
            omega_y: self.omega_y.clone(),
            omega_z: self.omega_z.clone(),
        }
    }
}

impl<T> Debug for GyroscopeReading<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("GyroscopeReading")
            .field(&self.omega_x)
            .field(&self.omega_y)
            .field(&self.omega_z)
            .finish()
    }
}

impl<T> Mul<T> for GyroscopeReading<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = GyroscopeReading<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            omega_x: self.omega_x * rhs.clone(),
            omega_y: self.omega_y * rhs.clone(),
            omega_z: self.omega_z * rhs.clone(),
        }
    }
}

impl<T> Sub<T> for GyroscopeReading<T>
where
    T: Sub<T, Output = T> + Clone,
{
    type Output = GyroscopeReading<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Self {
            omega_x: self.omega_x - rhs.clone(),
            omega_y: self.omega_y - rhs.clone(),
            omega_z: self.omega_z - rhs.clone(),
        }
    }
}

#[cfg(feature = "coordinate-frame")]
#[cfg_attr(docsrs, doc(cfg(feature = "coordinate-frame")))]
impl<T, C> From<C> for GyroscopeReading<T>
where
    C: coordinate_frame::CoordinateFrame<Type = T>,
    T: Copy + coordinate_frame::SaturatingNeg<Output = T>,
{
    fn from(value: C) -> Self {
        Self::north_east_down(value.to_ned())
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::Index<usize> for GyroscopeReading<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.omega_x,
            1 => &self.omega_y,
            2 => &self.omega_z,
            _ => panic!("Index out of bounds"),
        }
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::IndexMut<usize> for GyroscopeReading<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.omega_x,
            1 => &mut self.omega_y,
            2 => &mut self.omega_z,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl_standard_traits!(GyroscopeReading, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = GyroscopeReading::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = GyroscopeReading::<f32> {
            omega_x: 1.0,
            omega_y: 2.0,
            omega_z: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
