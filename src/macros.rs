#[macro_export(local_inner_macros)]
macro_rules! impl_standard_traits {
    ($type_name:ident, $type_param:ident) => {
        impl<$type_param> Copy for $type_name<$type_param> where $type_param: Copy {}

        #[cfg(feature = "micromath")]
        #[cfg_attr(docsrs, doc(cfg(feature = "micromath")))]
        impl<$type_param> From<$type_name<$type_param>> for micromath::vector::Vector3d<$type_param>
        where
            $type_param: micromath::vector::Component,
        {
            fn from(value: $type_name<$type_param>) -> micromath::vector::Vector3d<$type_param> {
                micromath::vector::Vector3d {
                    x: value[0],
                    y: value[1],
                    z: value[2],
                }
            }
        }
    };
}
