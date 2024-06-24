use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use core::ops::{Mul, Sub};

#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct GyroscopeBias<T> {
    /// The angular rate around the x-axis, in radians per second.
    pub omega_x: T,
    /// The angular rate around the y-axis, in radians per second.
    pub omega_y: T,
    /// The angular rate around the z-axis, in radians per second.
    pub omega_z: T,
}

impl<T> GyroscopeBias<T> {
    /// Initializes a new [`GyroscopeBias`] instance.
    #[inline(always)]
    pub const fn new(omega_x: T, omega_y: T, omega_z: T) -> Self {
        Self {
            omega_x,
            omega_y,
            omega_z,
        }
    }

    /// Returns the length of the [`GyroscopeBias`] vector.
    #[inline(always)]
    #[allow(unused)]
    pub const fn len(&self) -> usize {
        3
    }
}

impl<T> Default for GyroscopeBias<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for GyroscopeBias<T>
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

impl<T> Debug for GyroscopeBias<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("GyroscopeBias")
            .field(&self.omega_x)
            .field(&self.omega_y)
            .field(&self.omega_z)
            .finish()
    }
}

impl<T> Mul<T> for GyroscopeBias<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = GyroscopeBias<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            omega_x: self.omega_x * rhs.clone(),
            omega_y: self.omega_y * rhs.clone(),
            omega_z: self.omega_z * rhs.clone(),
        }
    }
}

impl<T> Sub<T> for GyroscopeBias<T>
where
    T: Sub<T, Output = T> + Clone,
{
    type Output = GyroscopeBias<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Self {
            omega_x: self.omega_x - rhs.clone(),
            omega_y: self.omega_y - rhs.clone(),
            omega_z: self.omega_z - rhs.clone(),
        }
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::Index<usize> for GyroscopeBias<T> {
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
impl<T> core::ops::IndexMut<usize> for GyroscopeBias<T> {
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

impl_standard_traits!(GyroscopeBias, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = GyroscopeBias::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = GyroscopeBias::<f32> {
            omega_x: 1.0,
            omega_y: 2.0,
            omega_z: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
