// Copyright 2021 The AutoCore.AI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
