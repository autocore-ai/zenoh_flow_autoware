use std::collections::HashMap;
use proc_macro::TokenStream;
use quote::quote;
use syn::{AttributeArgs, DeriveInput, ItemStruct, LitStr, Meta, NestedMeta, parse_macro_input};

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
                Ok(State::from(NativeNodeInstance { ptr: init(&get_config(cfg)) }))
            }
            fn finalize(&self, _state: &mut State) -> ZFResult<()> {
                Ok(())
            }
        }
    };
    gen.into()
}

#[proc_macro_derive(DefaultConfig)]
pub fn default_config_derive(_input: TokenStream) -> TokenStream {
    {
        quote! {
            impl Default for NativeConfig {
                fn default()->NativeConfig{
                    NativeConfig{node_name: String::from(env!("CARGO_PKG_NAME"))}
                }
            }
        }
    }
    .into()
}


#[proc_macro_derive(DefaultSendAndSync)]
pub fn zf_send_sync_derive(input: TokenStream) -> TokenStream {
    let ast = parse_macro_input!(input as DeriveInput);
    let ident = &ast.ident;
    let gen = quote! {
        unsafe impl Send for #ident {}
        unsafe impl Sync for #ident {}
    };
    gen.into()
}


#[proc_macro_attribute]
pub fn zf_default_node(attr: TokenStream, item: TokenStream) -> TokenStream {
    if attr.clone().into_iter().next().is_none() {
        panic!("#[zf_default_node(..)] init fn missing");
    }
    let in_args = parse_macro_input!(attr as AttributeArgs);
    let key_value_hash = _get_key_value(&in_args);
    let init_fn = key_value_hash.get("init_fn").unwrap();
    let init_fn_ident = syn::Ident::new(&*init_fn.value(), init_fn.span());
    let in_struct = parse_macro_input!(item as ItemStruct);
    let ident = &in_struct.ident;
    let gen = quote! {
        #in_struct
        
        impl Node for #ident {
            fn initialize(&self, cfg: &Option<Configuration>) -> ZFResult<State> {
                Ok(State::from(NativeNodeInstance { ptr: #init_fn_ident(&get_config(cfg)) }))
            }
            fn finalize(&self, _state: &mut State) -> ZFResult<()> {
                Ok(())
            }
        }
    };
    gen.into()
}

fn _get_key_value(nested_meta_list: &Vec<NestedMeta>) -> HashMap<String, &LitStr> {
    let mut key_value_hash: HashMap<String, &LitStr> = HashMap::new();
    for nested_meta in nested_meta_list.iter() {
        if let NestedMeta::Meta(Meta::NameValue(meta_name_value)) = nested_meta {
            let key = &meta_name_value.path.get_ident().unwrap().to_string();

            let lit_struct = &meta_name_value.lit;
            if let syn::Lit::Str(lit_str) = lit_struct {
                key_value_hash.insert(key.clone(), lit_str);
            } else {
                panic!("key: {:?} need type LitStr", key)
            }
        } else {
            panic!("Wrong type, \"MetaNameValue\" type needed")
        }
    }
    key_value_hash
}
