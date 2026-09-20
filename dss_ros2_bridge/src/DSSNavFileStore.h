#pragma once
 
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
 
// Linux / WSL2, C++17. Writes a complete batch into a fixed session directory,
// replacing whatever was saved there before (no more randomly-suffixed
// "session_XXXXXX" folders per save).
namespace dss_nav {
namespace fs = std::filesystem;
using File = std::pair<std::string, std::string>; // basename, bytes
 
// Change this to rename the fixed folder that every Start Mapping / Send Map
// save lands in (still created under storage_directory_).
inline constexpr const char* kFixedSessionDirName = "current";
 
inline void ValidateFilename(const std::string& name) {
    if (name.empty() || name == "." || name == ".." || name.size() > 200 ||
        name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
        throw std::invalid_argument("Expected a filename, not a path: " + name);
    }
    for (unsigned char c : name) {
        if (c < 32 || c == 127) throw std::invalid_argument("Control character in filename");
    }
}
 
inline fs::path SaveFiles(const fs::path& root, const std::vector<File>& files) {
    if (!root.is_absolute()) throw std::invalid_argument("storage_directory must be absolute");
    if (files.empty()) throw std::invalid_argument("No files to save");
    std::set<std::string> names;
    for (const auto& file : files) {
        ValidateFilename(file.first);
        if (!names.insert(file.first).second) throw std::invalid_argument("Duplicate filename: " + file.first);
    }
    fs::create_directories(root);
 
    // Write into a temporary staging directory first (mkdtemp gives us an
    // atomic, collision-free place to build the batch), then atomically swap
    // it into the fixed destination. This keeps the "no partial writes ever
    // visible" property of the original code while making the final folder
    // name stable instead of randomized.
    std::string pattern = (root / ".pending_XXXXXX").string();
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    char* directory = ::mkdtemp(writable.data());
    if (!directory) throw std::runtime_error("Cannot create temporary session directory");
    const fs::path pending(directory);
    try {
        for (const auto& file : files) {
            std::ofstream stream;
            stream.exceptions(std::ios::failbit | std::ios::badbit);
            stream.open(pending / file.first, std::ios::binary | std::ios::trunc);
            stream.write(file.second.data(), static_cast<std::streamsize>(file.second.size()));
            stream.close();
        }
        const fs::path destination = root / kFixedSessionDirName;
        // fs::rename() cannot replace a non-empty directory on Linux, so the
        // previous save (if any) is removed first. There's a brief window
        // here where "current" doesn't exist; that's an acceptable trade-off
        // for a fixed, overwritten-in-place folder name.
        std::error_code ignored;
        fs::remove_all(destination, ignored);
        fs::rename(pending, destination);
        return destination;
    } catch (...) {
        std::error_code ignored;
        fs::remove_all(pending, ignored);
        throw;
    }
}
} // namespace dss_nav
 