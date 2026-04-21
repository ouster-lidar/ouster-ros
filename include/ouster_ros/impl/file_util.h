#pragma once

#include <string>

namespace ouster_ros {
namespace impl {

std::string read_text_file(const std::string& file_path);

bool write_text_to_file(const std::string& file_path,
                        const std::string& text);

}   // namespace impl   
}   // namespace ouster_ros
