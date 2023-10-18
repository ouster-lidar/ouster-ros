/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_meta_helpers.h
 * @brief Provides a set of templated routines to enumerate or manipulate
 * various pcl point representations at compile time.
 */

#pragma once

#include <type_traits>
#include <string>
#include <pcl_conversions/pcl_conversions.h>

namespace ouster_ros {

#define DEFINE_MEMBER_CHECKER(member) \
    template<typename T, typename = void> \
    struct has_##member : std::false_type { }; \
    template<typename T> \
    struct has_##member<T, \
        typename std::enable_if< \
            !std::is_same<decltype(T::member), void>::value, \
            void \
            >::type \
        > : std::true_type { }; \
    template<typename C> \
    inline constexpr bool has_##member##_v = has_##member<C>::value;

template<typename T> inline void unused_variable(const T&) {}

template <typename T>
inline constexpr std::size_t point_element_count(const T& point) {
  return std::tuple_size<decltype(point.as_tuple())>::value;
}

template <>
inline constexpr std::size_t point_element_count<pcl::PointXYZ>(const pcl::PointXYZ&) { return 3U; }

template <>
inline constexpr std::size_t point_element_count<pcl::PointXYZI>(const pcl::PointXYZI&) { return 4U; }

// generic accessor that avoid having to type template before get
template <size_t I, typename T> 
inline constexpr auto& point_element_get(T& point) { return point.template get<I>(); }

// pcl::PointXYZ compile time element accessors
template <>
inline constexpr auto& point_element_get<0, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.x; }
template <>
inline constexpr auto& point_element_get<1, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.y; }
template <>
inline constexpr auto& point_element_get<2, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.z; }

// pcl::PointXYZI  compile time element accessors
template <>
inline  constexpr auto& point_element_get<0, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.x; }
template <>
inline  constexpr auto& point_element_get<1, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.y; }
template <>
inline  constexpr auto& point_element_get<2, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.z; }
template <>
inline  constexpr auto& point_element_get<3, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.intensity; }

// Apply an lambda function to the elements of a point in sequence 
template <std::size_t Index, std::size_t N, typename PointT, typename UnaryOp>
constexpr void iterate_point(PointT& point, UnaryOp unary_op) {
    if constexpr (Index < N) {
        unary_op(point_element_get<Index>(point));
        iterate_point<Index + 1, N, PointT, UnaryOp>(point, unary_op);
    } else {
      unused_variable(unary_op);
    }
}

// Apply an lambda function to the elements of a point in sequence passing
// in the index of the element as the first parameter along with each element 
template <std::size_t Index, std::size_t N, typename PointT, typename EnumOp>
constexpr void enumerate_point(PointT& point, EnumOp enum_op) {
    if constexpr (Index < N) {
        enum_op(Index, point_element_get<Index>(point));
        enumerate_point<Index + 1, N, PointT, EnumOp>(point, enum_op);
    } else {
      unused_variable(enum_op);
    }
}

template <bool Enable>
struct CondBinaryOp {
    template <typename A, typename B, typename BinaryOp>
    inline static void run(A& a, B& b, BinaryOp binary_op) {
        if constexpr (Enable) {
            binary_op(a, b);
        } else {
            unused_variable(a); unused_variable(b); unused_variable(binary_op);
        }
    }
};

template <bool Enable>
struct CondBinaryBind {
    template <typename A, typename B>
    inline static auto& run(A& a, B& b) {
        if constexpr (Enable) {
            unused_variable(b);
            return a;
        } else {
            unused_variable(a);
            return b;
        }
    }
};

}   // ouster_ros