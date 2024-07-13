use crate::impl_standard_traits;
use core::fmt::{Debug, Formatter};
use uniform_array_derive::UniformArray;

#[derive(UniformArray)]
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
