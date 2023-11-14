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

/**
 * @brief A macro that facilitate providing a compile time check if a struct has
 * a member with a specific name
 */
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

/**
 * @brief A template that can be used to silence unused varaiables warning.
 */
template<typename T> inline void unused_variable(const T&) {}

/**
 * @brief A struct that allows enabling or disabling the execution of an operation
 * using a compile time condition.
 */
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

/**
 * @brief A struct that a reference to the first variable if the compile time
 * condition was true and returns a reference to the second variable if condition
 * was false.
 */
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

namespace point {

/**
 * @brief A compile-time function to retrieve the number of elements that a 
 * certain pcl point type has
 * @param[in] point a pcl point type
 * @return the number of elements that a point has
 */
template <typename T>
inline constexpr std::size_t size(const T& point) {
  return std::tuple_size<decltype(point.as_tuple())>::value;
}

template <>
inline constexpr std::size_t size<pcl::PointXYZ>(const pcl::PointXYZ&) { return 3U; }

template <>
inline constexpr std::size_t size<pcl::PointXYZI>(const pcl::PointXYZI&) { return 4U; }

// generic accessor that avoid having to type template before get
template <size_t I, typename T> 
inline constexpr auto& get(T& point) { return point.template get<I>(); }

// pcl::PointXYZ compile time element accessors
template <>
inline constexpr auto& get<0, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.x; }
template <>
inline constexpr auto& get<1, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.y; }
template <>
inline constexpr auto& get<2, pcl::PointXYZ>(pcl::PointXYZ& point) { return point.z; }

// pcl::PointXYZI  compile time element accessors
template <>
inline  constexpr auto& get<0, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.x; }
template <>
inline  constexpr auto& get<1, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.y; }
template <>
inline  constexpr auto& get<2, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.z; }
template <>
inline  constexpr auto& get<3, pcl::PointXYZI>(pcl::PointXYZI& point) { return point.intensity; }

// TODO: create a generalized vardiac templates of apply and enumerate functions

/**
 * @brief Iterates the elements of a point (compile time) applying a lambda
 *  function to each element in sequence within the range [Index, N) where:
 *      `Index < N and N <= size(pt)`
 */
template <std::size_t Index, std::size_t N, typename PointT, typename UnaryOp>
constexpr void apply(PointT& pt, UnaryOp unary_op) {
    if constexpr (Index < N) {
        unary_op(get<Index>(pt));
        apply<Index + 1, N, PointT, UnaryOp>(pt, unary_op);
    } else {
      unused_variable(unary_op);
    }
}

/**
 * @brief Enumerates the elements of a point (compile time) applying a lambda
 * function to each element in sequence within the range [Index, N) where:
 *      `Index < N and N <= size(pt)`
 * The lambda function recieves the index of each element as the first parameter
 * and a reference to the element as the second parameter
 */
template <std::size_t Index, std::size_t N, typename PointT, typename EnumOp>
constexpr void enumerate(PointT& pt, EnumOp enum_op) {
    if constexpr (Index < N) {
        enum_op(Index, get<Index>(pt));
        enumerate<Index + 1, N, PointT, EnumOp>(pt, enum_op);
    } else {
      unused_variable(enum_op);
    }
}

}   // point
}   // ouster_ros