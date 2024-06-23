use crate::accelerometer_reading::AccelerometerReading;
use crate::impl_standard_traits;
use crate::magnetometer_reading::MagnetometerReading;
use core::borrow::Borrow;
use core::fmt::{Debug, Formatter};
use core::ops::{Add, Mul, Sub};
use minikalman::matrix::MatrixDataType;

#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3<T> {
    /// Initializes a new [`Vector3`] instance.
    #[inline(always)]
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl<T> Vector3<T> {
    /// Calculates the squared vector length.
    #[inline(always)]
    #[doc(alias = "length")]
    pub fn norm_sq(&self) -> T
    where
        T: Clone + Mul<T, Output = T> + Add<T, Output = T>,
    {
        (self.x.clone() * self.x.clone())
            + (self.y.clone() * self.y.clone())
            + (self.z.clone() * self.z.clone())
    }

    /// Calculates the vector length, i.e. its norm.
    #[inline(always)]
    pub fn norm(&self) -> T
    where
        T: MatrixDataType,
    {
        self.norm_sq().square_root()
    }

    /// Returns a normalized version of the vector.
    pub fn normalized(&self) -> Self
    where
        T: MatrixDataType,
    {
        let norm_inv = self.norm().recip();
        Self {
            x: self.x * norm_inv,
            y: self.y * norm_inv,
            z: self.z * norm_inv,
        }
    }

    /// Calculates the 3D vector cross product.
    pub fn cross<V>(&self, rhs: V) -> Vector3<T>
    where
        T: MatrixDataType + Sub<Output = T> + Mul<Output = T>,
        V: Borrow<Vector3<T>>,
    {
        let rhs = rhs.borrow();
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}

impl<T> Default for Vector3<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for Vector3<T>
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

impl<T> Debug for Vector3<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Vector3")
            .field(&self.x)
            .field(&self.y)
            .field(&self.z)
            .finish()
    }
}

impl<T> From<&AccelerometerReading<T>> for Vector3<T>
where
    T: Clone,
{
    #[inline]
    fn from(value: &AccelerometerReading<T>) -> Self {
        Self {
            x: value.x.clone(),
            y: value.y.clone(),
            z: value.z.clone(),
        }
    }
}

impl<T> From<AccelerometerReading<T>> for Vector3<T> {
    #[inline]
    fn from(value: AccelerometerReading<T>) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl<T> From<&MagnetometerReading<T>> for Vector3<T>
where
    T: Clone,
{
    #[inline]
    fn from(value: &MagnetometerReading<T>) -> Self {
        Self {
            x: value.x.clone(),
            y: value.y.clone(),
            z: value.z.clone(),
        }
    }
}

impl<T> From<MagnetometerReading<T>> for Vector3<T> {
    #[inline]
    fn from(value: MagnetometerReading<T>) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

/// Implements the vector dot product.
impl<T> Mul<Vector3<T>> for Vector3<T>
where
    T: Mul<T, Output = T> + Clone + Add<T, Output = T>,
{
    type Output = T;

    /// Calculates the inner product, also known as dot product.
    #[inline]
    fn mul(self, rhs: Vector3<T>) -> Self::Output {
        (self.x * rhs.x) + (self.y * rhs.y) + (self.z * rhs.z)
    }
}

impl<T> Mul<T> for Vector3<T>
where
    T: Mul<T, Output = T> + Clone,
{
    type Output = Vector3<T>;

    #[inline]
    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs.clone(),
            y: self.y * rhs.clone(),
            z: self.z * rhs.clone(),
        }
    }
}

impl<T> Sub<Vector3<T>> for Vector3<T>
where
    T: Sub<T, Output = T> + Clone,
{
    type Output = Vector3<T>;

    #[inline]
    fn sub(self, rhs: Vector3<T>) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T> Sub<T> for Vector3<T>
where
    T: Sub<T, Output = T> + Clone,
{
    type Output = Vector3<T>;

    #[inline]
    fn sub(self, rhs: T) -> Self::Output {
        Self {
            x: self.x - rhs.clone(),
            y: self.y - rhs.clone(),
            z: self.z - rhs.clone(),
        }
    }
}

#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::Index<usize> for Vector3<T> {
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
impl<T> core::ops::IndexMut<usize> for Vector3<T> {
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

impl_standard_traits!(Vector3, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_index() {
        let vec = Vector3::<f32> {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert_eq!(vec[0], 1.0);
        assert_eq!(vec[1], 2.0);
        assert_eq!(vec[2], 3.0);
    }

    #[test]
    fn test_length() {
        let vec = Vector3::<f32> {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        assert_eq!(vec.norm_sq(), 14.0);
        assert_eq!(vec.norm(), 14.0_f32.sqrt());
    }
}
