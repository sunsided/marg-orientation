pub trait GimbalLockZenithNadir<T> {
    /// The value for the zenith, i.e. π/2;
    const ZENITH: T;

    /// The value for the nadir, i.e. -π/2;
    const NADIR: T;
}

pub trait DetectGimbalLock<T>: GimbalLockZenithNadir<T> {
    /// Determines whether a Gimbal Lock situation is about to occur
    /// because the angle (provided in radians) is close to π/2 or -π/2.
    ///
    /// ## Arguments
    /// * `tolerance` - The tolerance in radians, e.g. 0.01 rad.
    fn close_to_zenith_or_nadir(&self, tolerance: T) -> bool;
}

pub trait IsNaN {
    /// Determines whether this value represents "not a number" (NaN).
    fn is_nan(&self) -> bool;
}

pub trait NormalizeAngle<T = Self> {
    type Output;

    /// Normalizes the angle into the range -π to π.
    fn normalize_angle(self) -> Self::Output;
}

pub trait ArcTan<T = Self> {
    type Output;

    fn atan2(self, rhs: T) -> Self::Output;
}

pub trait ArcSin<T = Self> {
    type Output;

    fn arcsin(self) -> Self::Output;
}

pub trait Abs<T = Self> {
    type Output;

    fn abs(self) -> Self::Output;
}

impl IsNaN for f32 {
    #[inline(always)]
    fn is_nan(&self) -> bool {
        (*self).is_nan()
    }
}

impl IsNaN for f64 {
    #[inline(always)]
    fn is_nan(&self) -> bool {
        (*self).is_nan()
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl NormalizeAngle<f32> for f32 {
    type Output = f32;

    fn normalize_angle(self) -> Self::Output {
        let mut normalized = self;
        while normalized > std::f32::consts::PI {
            normalized -= 2.0 * std::f32::consts::PI;
        }
        while normalized < -std::f32::consts::PI {
            normalized += 2.0 * std::f32::consts::PI;
        }
        normalized
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl NormalizeAngle<f64> for f64 {
    type Output = f64;

    fn normalize_angle(self) -> Self::Output {
        let mut normalized = self;
        while normalized > std::f64::consts::PI {
            normalized -= std::f64::consts::TAU;
        }
        while normalized < -std::f64::consts::PI {
            normalized += std::f64::consts::TAU;
        }
        normalized
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl Abs<f32> for f32 {
    type Output = f32;

    #[inline(always)]
    fn abs(self) -> Self::Output {
        f32::abs(self)
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl ArcTan<f32> for f32 {
    type Output = f32;

    #[inline(always)]
    fn atan2(self, other: f32) -> Self::Output {
        f32::atan2(self, other)
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl ArcTan<f64> for f64 {
    type Output = f64;

    #[inline(always)]
    fn atan2(self, other: f64) -> Self::Output {
        f64::atan2(self, other)
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl ArcSin<f32> for f32 {
    type Output = f32;

    #[inline(always)]
    fn arcsin(self) -> Self::Output {
        f32::asin(self)
    }
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl ArcSin<f64> for f64 {
    type Output = f64;

    #[inline(always)]
    fn arcsin(self) -> Self::Output {
        f64::asin(self)
    }
}

impl GimbalLockZenithNadir<f32> for f32 {
    const ZENITH: f32 = core::f32::consts::FRAC_PI_2;
    const NADIR: f32 = -core::f32::consts::FRAC_PI_2;
}

impl GimbalLockZenithNadir<f64> for f64 {
    const ZENITH: f64 = core::f64::consts::FRAC_PI_2;
    const NADIR: f64 = -core::f64::consts::FRAC_PI_2;
}

#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[cfg(feature = "std")]
impl<T> DetectGimbalLock<T> for T
where
    T: Copy + num_traits::Float + GimbalLockZenithNadir<T>,
{
    #[inline]
    fn close_to_zenith_or_nadir(&self, tolerance: T) -> bool {
        (*self - T::ZENITH).abs() <= tolerance || (*self - T::NADIR).abs() <= tolerance
    }
}

#[cfg(test)]
#[cfg(feature = "std")]
mod tests {
    use super::*;

    #[test]
    fn test_gimbal_lock_f32() {
        // The detection tolerance in radians.
        const TOLERANCE: f32 = 0.1;

        // The value to use for testing the tolerance. We use a value less than
        // the tolerance here to account for floating-point rounding issues.
        const TOLERANCE_TEST: f32 = TOLERANCE * 0.99;

        assert!(!0.0_f32.close_to_zenith_or_nadir(TOLERANCE));
        assert!(!1.0_f32.close_to_zenith_or_nadir(TOLERANCE));
        assert!(!(-1.0_f32).close_to_zenith_or_nadir(TOLERANCE));

        assert!(core::f32::consts::FRAC_PI_2.close_to_zenith_or_nadir(TOLERANCE));
        assert!((-core::f32::consts::FRAC_PI_2).close_to_zenith_or_nadir(TOLERANCE));

        assert!((core::f32::consts::FRAC_PI_2 + TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE));
        assert!((core::f32::consts::FRAC_PI_2 - TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE));
        assert!(
            (-core::f32::consts::FRAC_PI_2 + TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE)
        );
        assert!(
            (-core::f32::consts::FRAC_PI_2 - TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE)
        );
    }

    #[test]
    fn test_gimbal_lock_f64() {
        // The detection tolerance in radians.
        const TOLERANCE: f64 = 0.1;

        // The value to use for testing the tolerance. We use a value less than
        // the tolerance here to account for floating-point rounding issues.
        const TOLERANCE_TEST: f64 = TOLERANCE * 0.99;

        assert!(!0.0_f64.close_to_zenith_or_nadir(TOLERANCE));
        assert!(!1.0_f64.close_to_zenith_or_nadir(TOLERANCE));
        assert!(!(-1.0_f64).close_to_zenith_or_nadir(TOLERANCE));

        assert!(core::f64::consts::FRAC_PI_2.close_to_zenith_or_nadir(TOLERANCE));
        assert!((-core::f64::consts::FRAC_PI_2).close_to_zenith_or_nadir(TOLERANCE));

        assert!((core::f64::consts::FRAC_PI_2 + TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE));
        assert!((core::f64::consts::FRAC_PI_2 - TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE));
        assert!(
            (-core::f64::consts::FRAC_PI_2 + TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE)
        );
        assert!(
            (-core::f64::consts::FRAC_PI_2 - TOLERANCE_TEST).close_to_zenith_or_nadir(TOLERANCE)
        );
    }
}
