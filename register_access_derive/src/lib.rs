#[macro_use]
extern crate macro_rules_attribute;
use paste::paste;

macro_rules! RegistersByConfig {
    (
        $( #[$attr:meta] )*
            $pub:vis
            struct $NewType:ident {
                $(
                $(#[$field_attr:meta])*
                $fvis:vis $field:ident : $ty:ty,
                )*
            }
        ) => {
        paste! {
            impl $NewType {
                $(
                    #[inline]
                    async fn [<$field _set>]<T>(self: $NewType, interface: T) {
                        interface.write_register(
                            [<$field:camel>]::default().with_value(self.$field.into())
                        )
                    }
                )*
            }
        }
    };
}
use RegistersByConfig;

#[cfg(test)]
mod tests {
    macro_rules! Into {
        (
            $( #[$attr:meta] )*
            $pub:vis
            struct $NewType:ident (
                $(#[$field_attr:meta])*
                $field_pub:vis
                $Inner:ty $(,

                $($rest:tt)* )?
            );
        ) => {
            impl ::core::convert::Into<$Inner> for $NewType {
                #[inline]
                fn into(self: $NewType) -> $Inner {
                    self.0
                }
            }
        };
    }
    use Into;

    macro_rules! From {
        (
            $( #[$attr:meta] )*
            $pub:vis
            struct $NewType:ident (
                $(#[$field_attr:meta])*
                $field_pub:vis
                $Inner:ty $(,

                $(#[$other_field_attr:meta])*
                $other_field_pub:vis
                $Rest:ty )* $(,)?
            );
        ) => (
            impl ::core::convert::From<$Inner> for $NewType {
                #[inline]
                fn from (inner: $Inner)
                -> Self
                {
                    Self(inner, $($Rest::default),*)
                }
            }
        )}
    use From;

    use super::*;

    #[test]
    fn it_works() {
        #[derive(Into!, From!)]
        pub struct PlayerId(pub usize);
    }

    #[test]
    fn real_macro() {
        trait interface {
            fn write_register
        }
        struct Bar {}
        impl Bar {
            fn default() -> Bar {Bar {}}
            fn with_value(self, b: u8) {}
        }
        #[derive(RegistersByConfig!)]
        pub struct Foo {
            bar: u8,
        }
    }
}
