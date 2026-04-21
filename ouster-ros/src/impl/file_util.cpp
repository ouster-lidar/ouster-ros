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

} // namespace impl
} // namespace ouster_ros