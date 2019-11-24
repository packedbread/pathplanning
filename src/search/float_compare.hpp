#pragma once
#include <cmath>
#include <type_traits>
#include <limits>

namespace planner {
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool very_close_equals(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return std::abs(u - v) < comparison_epsilon * std::abs(u) && std::abs(u - v) < comparison_epsilon * std::abs(v);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool very_close_less(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return u - v < -comparison_epsilon * std::abs(u) && u - v < -comparison_epsilon * std::abs(v);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool very_close_greater(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return u - v > comparison_epsilon * std::abs(u) && u - v > comparison_epsilon * std::abs(v);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool close_enough_equals(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return std::abs(u - v) < comparison_epsilon * std::abs(u) || std::abs(u - v) < comparison_epsilon * std::abs(v);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool close_enough_less(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return u - v < -comparison_epsilon * std::abs(u) || u - v < -comparison_epsilon * std::abs(v);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
    bool close_enough_greater(T u, T v, T comparison_epsilon = std::numeric_limits<T>::epsilon()) {
        return u - v > comparison_epsilon * std::abs(u) || u - v > comparison_epsilon * std::abs(v);
    }
}