#include <vector>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <limits>

using RealPointF = Eigen::Array<float, -1, 3, Eigen::RowMajor>;

#pragma pack(push, 1)
struct SrcColumn {
    uint64_t timestamp;
    uint16_t measurement_id;
    uint16_t status : 1;
    uint16_t padding : 15;
};

struct Dual {
    uint32_t rng1 : 19;
    uint32_t pad1 : 5;
    uint32_t ref1 : 8;
    uint32_t rng2 : 19;
    uint32_t pad2 : 5;
    uint32_t ref2 : 8;
    uint16_t sig1;
    uint16_t sig2;
    uint16_t nir;
    uint16_t pad3;
    
    const static int HeaderSize = 256/8;
    
    template <int ret>
    auto get_range()
    {
        return ret ? rng2 : rng1;
    }
    
    template <int ret>
    auto get_sig()
    {
        return ret ? sig2 : sig1;
    }
    
    template <int ret>
    auto get_refl()
    {
        return ret ? ref2 : ref1;
    }
    
    auto get_nir()
    {
        return nir;
    }
    
    constexpr static bool is_dual()
    {
        return true;
    }
};

struct LowDataRate {
    uint16_t rng1 : 15;
    uint16_t pad1 : 1;
    uint8_t ref1;
    uint8_t nir;
    
    const static int HeaderSize = 256/8;
    
    template <int ret>
    auto get_range()
    {
        // only support ret 0
        return rng1 * 8;
    }
    
    template <int ret>
    auto get_refl()
    {
        // only support ret 0
        return ref1;
    }
    
    template <int ret>
    auto get_sig()
    {
        throw std::runtime_error("SrcPoint2 has no signal information.");
        return 0;
    }
    
    auto get_nir()
    {
        return nir << 8;
    }
    
    constexpr static bool is_dual()
    {
        return false;
    }
};

struct Single {
    uint32_t rng1 : 19;
    uint32_t pad1 : 13;
    uint8_t ref1;
    uint8_t pad2;
    uint16_t sig1;
    uint16_t nir;
    uint16_t pad3;
    
    const static int HeaderSize = 256/8;
    
    template <int ret>
    auto get_range()
    {
        // only support ret 0
        return rng1;
    }
    
    template <int ret>
    auto get_refl()
    {
        // only support ret 0
        return ref1;
    }
    
    template <int ret>
    auto get_sig()
    {
        return sig1;
    }
    
    auto get_nir()
    {
        return nir;
    }
    
    constexpr static bool is_dual()
    {
        return false;
    }
};

#pragma pack(pop)

template <class A, class B, int ret>
bool DoReturn(A& src, B& dst, int col, int row, uint32_t ts, const float* origin, const float* dir) {
    float range = src.template get_range<ret>();
    
    const int min_range = 0;
    const int max_range = 1000000000;
    
    // ignore filtered out samples (non returns and out of range)
    if (range == 0 || range < min_range || range > max_range) {
        dst.x = std::numeric_limits<float>::quiet_NaN();
        dst.y = std::numeric_limits<float>::quiet_NaN();
        dst.z = std::numeric_limits<float>::quiet_NaN();
        
        // todo what to do with other fields?
        return false;
    }
    
    // calculate range
    dst.x = origin[0] + dir[0]*range;
    dst.y = origin[1] + dir[1]*range;
    dst.z = origin[2] + dir[2]*range;
    
    // now try and fill everything
    dst.template fill<A, ret>(src, row, ts);
    
    return true;
}

// tests doing the thing
template<class A, class B>
void CopyBytes(int cols, int rows,
              std::vector<uint8_t>& cloud, std::vector<uint8_t>& cloud2,
              const std::vector<uint8_t>& packet,
              const float* lut_dir, const float* lut_off) {
    
    // todo handle header
    
    // for now lets just pack it all in
    // todo determine
    uint64_t start_ts = 0;
    
    bool ordered = true;
    bool destagger = true;
    destagger &= ordered;
    
    auto& points = cloud;
    auto& points2 = cloud2;
    
    if (!ordered) {
        points.resize(sizeof(B));
        if (A::is_dual()) {
            points2.resize(sizeof(B));
        }
    } else {
        points.resize(1024*rows*sizeof(B));
        if (A::is_dual()) {
            points2.resize(1024*rows*sizeof(B));
        }
    }
        
    // todo fill out
    bool dense = true;
    
    int start_offset = A::HeaderSize;
    int column_offset = sizeof(SrcColumn) + sizeof(A)*rows;
    for (int c = 0; c < cols; c++) {
        auto column = (SrcColumn*)(packet.data() + c*column_offset + start_offset);
        if (!column->status) {
            continue;
        }
        auto cloud_col = column->measurement_id;
        auto ts = column->timestamp - start_ts;
        auto pixels = (A*)(packet.data() + c*column_offset + sizeof(SrcColumn) + start_offset);
        for (int r = 0; r < rows; r++) {
            A& src = pixels[r];
            
            auto dc = cloud_col;
            if (destagger) {
                dc = cloud_col + 0;// todo get stagger
            }

            B* dst1 = (B*)&points[points.size() - sizeof(B)];
            if (ordered) {
                dst1 = (B*)&points[(dc*rows + r)*sizeof(B)];
            }
            
            auto off = &lut_off[(r*1024 + dc)*3];
            auto dir = &lut_dir[(r*1024 + dc)*3];
            
            if (DoReturn<A, B, 0>(src, *dst1, c, r, ts, off, dir) && !ordered) {
                points.resize(points.size() + sizeof(B));
            }
            if (src.is_dual()) {
                B* dst2 = (B*)&points2[points2.size() - sizeof(B)];
                if (!ordered) {
                    dst2 = (B*)&points2[(dc*rows + r)*sizeof(B)];
                }
                if (DoReturn<A, B, 1>(src, *dst2, c, r, ts, off, dir) && !ordered) {
                    points2.resize(points2.size() + sizeof(B));
                }
            }
        }
    }
    if (!ordered) {
        points.resize(points.size() - sizeof(B));
        if (A::is_dual()) {
            points2.resize(points2.size() - sizeof(B));
        }
    }
}

// tests doing the thing
template<class A, class B>
void CopyReal(int cols, int rows,
              ouster_ros::Cloud<B>& cloud, ouster_ros::Cloud<B>& cloud2,
              const std::vector<uint8_t>& packet,
              const float* lut_dir, const float* lut_off) {
    
    // todo handle header
    
    // for now lets just pack it all in
    // todo determine
    uint64_t start_ts = 0;
    
    bool ordered = true;
    bool destagger = true;
    destagger &= ordered;
    
    auto& points = cloud.points;
    auto& points2 = cloud.points;
    
    if (!ordered) {
        points.emplace_back();
        if (A::is_dual()) {
            points2.emplace_back();
        }
    } else {
        points.resize(1024*rows);
        if (A::is_dual()) {
            points2.resize(1024*rows);
        }
    }
        
    // todo fill out
    bool dense = true;
    
    int start_offset = A::HeaderSize;
    int column_offset = sizeof(SrcColumn) + sizeof(A)*rows;
    for (int c = 0; c < cols; c++) {
        auto column = (SrcColumn*)(packet.data() + c*column_offset + start_offset);
        if (!column->status) {
            continue;
        }
        auto cloud_col = column->measurement_id;
        auto ts = column->timestamp - start_ts;
        auto pixels = (A*)(packet.data() + c*column_offset + sizeof(SrcColumn) + start_offset);
        for (int r = 0; r < rows; r++) {
            A& src = pixels[r];
            
            auto dc = cloud_col;
            if (destagger) {
                dc = cloud_col + 0;// todo get stagger
            }

            B* dst1 = &points.back();
            if (ordered) {
                dst1 = &points[dc*rows + r];
            }
            
            auto off = &lut_off[(r*1024 + dc)*3];
            auto dir = &lut_dir[(r*1024 + dc)*3];
            
            if (DoReturn<A, B, 0>(src, *dst1, c, r, ts, off, dir) && !ordered) {
                points.emplace_back();
            }
            if (src.is_dual()) {
                B* dst2 = &points2.back();
                if (!ordered) {
                    dst2 = &points2[dc*rows + r];
                }
                if (DoReturn<A, B, 1>(src, *dst2, c, r, ts, off, dir) && !ordered) {
                    points2.emplace_back();
                }
            }
        }
    }
    if (!ordered) {
        points.resize(points.size() - 1);
        if (A::is_dual()) {
            points2.resize(points2.size() - 1);
        }
    }
}
