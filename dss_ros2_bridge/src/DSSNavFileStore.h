#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Linux / WSL2, C++17. Writes a complete batch into a new session directory.
namespace dss_nav {
namespace fs = std::filesystem;
using File = std::pair<std::string, std::string>; // basename, bytes

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
        const auto suffix = pending.filename().string().substr(std::string(".pending_").size());
        const fs::path destination = root / ("session_" + suffix);
        if (fs::exists(destination)) throw std::runtime_error("Session directory collision");
        fs::rename(pending, destination);
        return destination;
    } catch (...) {
        std::error_code ignored;
        fs::remove_all(pending, ignored);
        throw;
    }
}
} // namespace dss_nav
