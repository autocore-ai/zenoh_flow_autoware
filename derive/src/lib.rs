use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

#[proc_macro_derive(ZFFakeSerialize)]
pub fn zf_fake_serialize_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);
    let ident = &ast.ident;
    let gen = quote! {
        impl zenoh_flow::ZFData for #ident {
            fn try_serialize(&self) -> zenoh_flow::ZFResult<Vec<u8>> {
                todo!()
            }
        }
        impl zenoh_flow::Deserializable for #ident {
            fn try_deserialize(_bytes: &[u8]) -> zenoh_flow::ZFResult<Self>
            where
            Self: Sized,
            {
                todo!();
            }
        }
    };
    gen.into()
}

#[proc_macro_derive(ZenohFlowNode)]
pub fn zf_node_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);
    let ident = &ast.ident;
    let gen = quote! {
        unsafe impl Send for #ident {}
        unsafe impl Sync for #ident {}

        impl Node for #ident {
            fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
                Ok(State::from(NativeNodeInstance { ptr: init(get_config(cfg).unwrap()) }))
            }
        }
    };
    gen.into()
}
