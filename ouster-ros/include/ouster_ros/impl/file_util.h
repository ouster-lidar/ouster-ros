#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ouster_ros {
namespace impl {

std::string read_text_file(const std::string& file_path);

bool write_text_to_file(const std::string& file_path,
                        const std::string& text);

// returns an empty vector if the file could not be opened
std::vector<uint8_t> read_binary_file(const std::string& file_path);

} // namespace impl
} // namespace ouster_ros