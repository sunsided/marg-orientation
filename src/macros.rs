#[macro_export(local_inner_macros)]
macro_rules! impl_standard_traits {
    ($type_name:ident, $type_param:ident) => {
        impl<$type_param> Copy for $type_name<$type_param> where $type_param: Copy {}
    };
}
