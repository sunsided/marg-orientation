#[macro_export]
macro_rules! impl_standard_traits {
    ($type_name:ident, $type_param:ident) => {
        impl<$type_param> Copy for $type_name<$type_param> where $type_param: Copy {}

        #[cfg_attr(docsrs, doc(cfg(feature = "unsafe")))]
        #[cfg(feature = "unsafe")]
        impl<$type_param> $type_name<$type_param>
        where
            $type_param: Default,
        {
            /// Constructs a new instance from a slice.
            #[allow(unused)]
            #[inline]
            pub fn from_slice(slice: &[$type_param]) -> &Self {
                assert_eq!(
                    slice.len(),
                    core::mem::size_of::<Self>() / core::mem::size_of::<$type_param>()
                );

                // SAFETY: $type_name only contains `$type_param` fields and is `repr(C)`
                unsafe { &*(slice.as_ptr() as *const Self) }
            }

            /// Constructs a new instance from a mutable slice.
            #[allow(unused)]
            #[inline]
            pub fn from_mut_slice(slice: &mut [$type_param]) -> &mut Self {
                assert_eq!(
                    slice.len(),
                    core::mem::size_of::<Self>() / core::mem::size_of::<$type_param>()
                );

                // SAFETY: $type_name only contains `$type_param` fields and is `repr(C)`
                unsafe { &mut *(slice.as_mut_ptr() as *mut Self) }
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unsafe")))]
        #[cfg(feature = "unsafe")]
        impl<$type_param> core::convert::AsRef<[$type_param]> for $type_name<$type_param>
        where
            $type_param: Default,
        {
            fn as_ref(&self) -> &[$type_param] {
                unsafe {
                    // SAFETY: $type_name only contains `$type_param` fields and is `repr(C)`
                    core::slice::from_raw_parts(
                        self as *const _ as *const $type_param,
                        core::mem::size_of::<$type_name<$type_param>>()
                            / core::mem::size_of::<$type_param>(),
                    )
                }
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unsafe")))]
        #[cfg(feature = "unsafe")]
        impl<$type_param> core::convert::AsMut<[$type_param]> for $type_name<$type_param>
        where
            $type_param: Default,
        {
            fn as_mut(&mut self) -> &mut [$type_param] {
                unsafe {
                    // SAFETY: $type_name only contains `$type_param` fields and is `repr(C)`
                    core::slice::from_raw_parts_mut(
                        self as *mut _ as *mut $type_param,
                        core::mem::size_of::<$type_name<$type_param>>()
                            / core::mem::size_of::<$type_param>(),
                    )
                }
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unsafe")))]
        #[cfg(feature = "unsafe")]
        impl<$type_param> core::ops::Deref for $type_name<$type_param>
        where
            $type_param: Default,
        {
            type Target = [$type_param];

            fn deref(&self) -> &Self::Target {
                self.as_ref()
            }
        }

        #[cfg_attr(docsrs, doc(cfg(feature = "unsafe")))]
        #[cfg(feature = "unsafe")]
        impl<$type_param> core::ops::DerefMut for $type_name<$type_param>
        where
            $type_param: Default,
        {
            fn deref_mut(&mut self) -> &mut Self::Target {
                self.as_mut()
            }
        }

        #[cfg(test)]
        paste::paste! {
            #[cfg(test)]
            mod [<tests_gen_ $type_name:lower>] {
                #[cfg(feature = "unsafe")]
                use super::*;

                #[test]
                #[cfg(feature = "unsafe")]
                fn test_from_slice() {
                    const TYPE_SIZE: usize = core::mem::size_of::<$type_name<u32>>();
                    const NUM_ELEMS: usize = TYPE_SIZE / core::mem::size_of::<u32>();
                    const ARRAY_SIZE: usize = NUM_ELEMS + 1;
                    let data = [0; ARRAY_SIZE];
                    let state = $type_name::from_slice(&data[..NUM_ELEMS]);
                    assert_eq!(state.len(), NUM_ELEMS);
                    assert!(core::ptr::eq(state.as_ptr(), data.as_ptr()));
                }

                #[test]
                #[cfg(feature = "unsafe")]
                fn test_from_slice_mut() {
                    const TYPE_SIZE: usize = core::mem::size_of::<$type_name<u32>>();
                    const NUM_ELEMS: usize = TYPE_SIZE / core::mem::size_of::<u32>();
                    const ARRAY_SIZE: usize = NUM_ELEMS + 1;
                    let mut data = [0; ARRAY_SIZE];
                    {
                        let state = $type_name::from_mut_slice(&mut data[..NUM_ELEMS]);
                        state[0] = 10;
                        assert!(core::ptr::eq(state.as_ptr(), data.as_ptr()));
                    }
                    assert_eq!(data[0], 10, "expect data to be changed");
                }
            }
        }
    };
}
