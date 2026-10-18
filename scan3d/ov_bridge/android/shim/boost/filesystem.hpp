// Android build shim: OpenVINS only uses exists/remove/path/create_directories from boost::filesystem.
// Boost.Filesystem needs a compiled library; std::filesystem (C++17, NDK libc++) has the same API for these.
#pragma once
#include <filesystem>
namespace boost {
namespace filesystem = std::filesystem;
}
