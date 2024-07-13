use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};

#[cfg_attr(test, ensure_uniform_type::ensure_uniform_type)]
#[repr(C)]
pub struct EulerAngles<T> {
    /// The roll angle, in radians.
    pub roll_phi: T,
    /// The pitch angle, in radians.
    pub pitch_theta: T,
    /// The yaw angle, in radians.
    pub yaw_psi: T,
}

impl<T> EulerAngles<T> {
    /// Initializes a new [`EulerAngles`] instance.
    #[inline(always)]
    pub const fn new(roll_phi: T, pitch_theta: T, yaw_psi: T) -> Self {
        Self {
            roll_phi,
            pitch_theta,
            yaw_psi,
        }
    }

    /// Returns the length of the [`EulerAngles`] vector.
    #[inline(always)]
    #[allow(unused)]
    #[allow(clippy::len_without_is_empty)]
    pub const fn len(&self) -> usize {
        3
    }
}

impl<T> Default for EulerAngles<T>
where
    T: Default,
{
    #[inline]
    fn default() -> Self {
        Self::new(Default::default(), Default::default(), Default::default())
    }
}

impl<T> Clone for EulerAngles<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            roll_phi: self.roll_phi.clone(),
            pitch_theta: self.pitch_theta.clone(),
            yaw_psi: self.yaw_psi.clone(),
        }
    }
}

impl<T> Debug for EulerAngles<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("EulerAngles")
            .field(&self.roll_phi)
            .field(&self.pitch_theta)
            .field(&self.yaw_psi)
            .finish()
    }
}

#[cfg_attr(docsrs, doc(cfg(not(feature = "unsafe"))))]
#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::Index<usize> for EulerAngles<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.roll_phi,
            1 => &self.pitch_theta,
            2 => &self.yaw_psi,
            _ => panic!("Index out of bounds"),
        }
    }
}

#[cfg_attr(docsrs, doc(cfg(not(feature = "unsafe"))))]
#[cfg(not(feature = "unsafe"))]
impl<T> core::ops::IndexMut<usize> for EulerAngles<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.roll_phi,
            1 => &mut self.pitch_theta,
            2 => &mut self.yaw_psi,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl_standard_traits!(EulerAngles, T);

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_len() {
        let reading = EulerAngles::<f32>::default();
        assert_eq!(reading.len(), 3);
    }

    #[test]
    fn test_index() {
        let reading = EulerAngles::<f32> {
            roll_phi: 1.0,
            pitch_theta: 2.0,
            yaw_psi: 3.0,
        };

        assert_eq!(reading[0], 1.0);
        assert_eq!(reading[1], 2.0);
        assert_eq!(reading[2], 3.0);
    }
}
