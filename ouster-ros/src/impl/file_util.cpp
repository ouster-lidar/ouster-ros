#include <ouster_ros/impl/file_util.h>
#include <fstream>
#include <sstream>


namespace ouster_ros {
namespace impl {

std::string read_text_file(const std::string& file_path) {
    std::ifstream ifs{};
    ifs.open(file_path);
    if (ifs.fail()) return {};
    std::stringstream buf;
    buf << ifs.rdbuf();
    return buf.str();
}

bool write_text_to_file(const std::string& file_path,
                        const std::string& text) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) return false;
    ofs << text << std::endl;
    ofs.close();
    return true;
}

std::vector<uint8_t> read_binary_file(const std::string& file_path) {
    std::ifstream ifs(file_path, std::ios::binary | std::ios::ate);
    if (ifs.fail()) return {};
    auto size = ifs.tellg();
    if (size <= 0) return {};
    std::vector<uint8_t> buf(static_cast<size_t>(size));
    ifs.seekg(0);
    ifs.read(reinterpret_cast<char*>(buf.data()), size);
    if (ifs.fail()) return {};
    return buf;
}

} // namespace impl
} // namespace ouster_ros