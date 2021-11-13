#pragma once
#include <memory>
namespace zenoh_flow
{
    namespace autoware_auto
    {
        namespace ffi
        {
            class NativeConfig
            {
            };
            std::unique_ptr<NativeConfig> native_config()
            {
                return std::make_unique<NativeConfig>();
            }
        }
    }
}
