#include <zenoh_flow_msg_convert/zenoh_flow_msg_convert.hpp>

BuiltinInterfacesTime Convert(const builtin_interfaces::msg::Time &src) { return {src.sec, src.nanosec}; }
builtin_interfaces::msg::Time Convert(const BuiltinInterfacesTime &src) { return builtin_interfaces::msg::Time().set__sec(src.sec).set__nanosec(src.nanosec); }
rust::cxxbridge1::String Convert(const std::string &src) { return rust::cxxbridge1::String(src.data(), src.size()); }
std::string Convert(const rust::cxxbridge1::String &src) { return std::string(src.data(), src.size()); }
