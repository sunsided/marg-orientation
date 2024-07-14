#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(feature = "unsafe", allow(unsafe_code))]
#![cfg_attr(not(feature = "unsafe"), forbid(unsafe_code))]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod gyro_drift;
pub mod gyro_free;
pub mod test_estimator;
pub mod types;

mod macros;
mod num_traits;

pub use crate::num_traits::*;

/// Imports commonly used types and traits.
pub mod prelude {
    pub use crate::num_traits::*;
    pub use crate::types::Vector3;
}
