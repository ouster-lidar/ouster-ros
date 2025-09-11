/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file diagnostics_visitor_registry.h
 * @brief Standalone visitor registry with observability API (no ouster SDK
 * dependencies)
 */

#pragma once

#include <diagnostic_msgs/msg/key_value.hpp>
#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>
#include <vector>

namespace ouster_ros
{

template <typename... MsgT>
struct DiagnosticsVisitorRegistry
{
public:
  // Public type definitions
  template <typename MessageType, typename VisitorFunc>
  struct visitor_registration
  {
    VisitorFunc visitor;
    explicit visitor_registration(VisitorFunc && v) : visitor(std::forward<VisitorFunc>(v)) {}
  };

  template <typename MessageType>
  using VisitorFunction =
    std::function<std::vector<diagnostic_msgs::msg::KeyValue>(const MessageType &)>;

  // Public static helper functions
  template <typename MessageType, typename VisitorFunc>
  static auto make_visitor(VisitorFunc && visitor)
  {
    return visitor_registration<MessageType, VisitorFunc>(std::forward<VisitorFunc>(visitor));
  }

  DiagnosticsVisitorRegistry() = default;

  template <typename... VisitorRegs>
  explicit DiagnosticsVisitorRegistry(VisitorRegs &&... visitor_regs)
  {
    (register_any_visitor(std::forward<VisitorRegs>(visitor_regs)), ...);
  }

  template <typename MessageType>
  std::vector<diagnostic_msgs::msg::KeyValue> operator()(const MessageType & msg) const
  {
    static_assert(
      (std::is_same_v<MessageType, MsgT> || ...),
      "MessageType must be one of the "
      "DiagnosticsVisitorRegistry template parameters");

    const auto & visitor_opt = get_visitor<MessageType>();

    if (visitor_opt.has_value()) {
      return visitor_opt.value()(msg);
    } else {
      return default_analysis<MessageType>();
    }
  }

  template <typename VisitorFunc>
  void register_visitor(VisitorFunc && visitor)
  {
    using MessageType = typename function_traits<std::decay_t<VisitorFunc>>::message_type;
    register_visitor_impl<MessageType>(std::forward<VisitorFunc>(visitor));
  }

  template <typename T>
  static constexpr bool supports()
  {
    return (std::is_same_v<T, MsgT> || ...);
  }

  template <typename T>
  bool has_visitor() const noexcept
  {
    static_assert(supports<T>(), "Type not registered");
    return get_visitor<T>().has_value();
  }

  template <typename Callable>
  void for_each_supported(Callable && c) const
  {
    (static_cast<void>(std::forward<Callable>(c).template operator()<MsgT>()), ...);
  }

  template <typename Callable>
  void for_each_registered(Callable && c) const
  {
    (void)std::initializer_list<int>{
      (get_visitor<MsgT>().has_value() ? (std::forward<Callable>(c).template operator()<MsgT>(), 0)
                                       : 0)...};
  }

private:
  std::tuple<std::optional<VisitorFunction<MsgT>>...> visitors_;

  // Private helper types for callable type deduction
  template <typename T>
  struct function_traits;
  // Function pointer
  template <typename R, typename Arg>
  struct function_traits<R (*)(Arg)>
  {
    using message_type = std::decay_t<Arg>;
  };
  // Member function pointer (const)
  template <typename R, typename C, typename Arg>
  struct function_traits<R (C::*)(Arg) const>
  {
    using message_type = std::decay_t<Arg>;
  };
  // Member function pointer (non-const)
  template <typename R, typename C, typename Arg>
  struct function_traits<R (C::*)(Arg)>
  {
    using message_type = std::decay_t<Arg>;
  };
  // std::function
  template <typename R, typename Arg>
  struct function_traits<std::function<R(Arg)>>
  {
    using message_type = std::decay_t<Arg>;
  };
  // Generic functor/lambda
  template <typename T>
  struct function_traits : function_traits<decltype(&T::operator())>
  {
  };

  template <typename MessageType, typename VisitorFunc>
  void register_visitor_impl(VisitorFunc && visitor)
  {
    static_assert(
      (std::is_same_v<MessageType, MsgT> || ...),
      "MessageType must be one of the "
      "DiagnosticsVisitorRegistry template parameters");

    using ReturnType = std::invoke_result_t<VisitorFunc, const MessageType &>;
    using ExpectedReturnType = std::vector<diagnostic_msgs::msg::KeyValue>;
    static_assert(
      std::is_convertible_v<ReturnType, ExpectedReturnType>,
      "Visitor must return std::vector<diagnostic_msgs::msg::KeyValue>");

    VisitorFunction<MessageType> typed_visitor = std::forward<VisitorFunc>(visitor);
    std::get<std::optional<VisitorFunction<MessageType>>>(visitors_) = std::move(typed_visitor);
  }

  template <typename MessageType>
  const std::optional<VisitorFunction<MessageType>> & get_visitor() const
  {
    static_assert(
      (std::is_same_v<MessageType, MsgT> || ...),
      "MessageType must be one of the "
      "DiagnosticsVisitorRegistry template parameters");

    return std::get<std::optional<VisitorFunction<MessageType>>>(visitors_);
  }

  template <typename MessageType, typename VisitorFunc>
  void register_from_helper(visitor_registration<MessageType, VisitorFunc> && reg)
  {
    register_visitor_impl<MessageType>(std::move(reg.visitor));
  }

  template <typename VisitorFunc>
  void register_from_lambda(VisitorFunc && visitor_func)
  {
    using MessageType = typename function_traits<std::decay_t<VisitorFunc>>::message_type;
    static_assert(
      (std::is_same_v<MessageType, MsgT> || ...),
      "Lambda message type must be one of the "
      "DiagnosticsVisitorRegistry template parameters");

    register_visitor_impl<MessageType>(std::forward<VisitorFunc>(visitor_func));
  }

  template <typename MessageType>
  const std::vector<diagnostic_msgs::msg::KeyValue> & default_analysis() const noexcept
  {
    const static std::vector<diagnostic_msgs::msg::KeyValue> empty;
    return empty;
  }

  template <typename MessageType, typename VisitorFunc>
  void register_any_visitor(visitor_registration<MessageType, VisitorFunc> && reg)
  {
    register_from_helper(std::move(reg));
  }

  template <typename VisitorFunc>
  void register_any_visitor(VisitorFunc && visitor_func)
  {
    register_visitor(std::forward<VisitorFunc>(visitor_func));
  }
};

template <typename... MsgT, typename... VisitorRegs>
DiagnosticsVisitorRegistry<MsgT...> make_visitor_registry(VisitorRegs &&... visitor_regs)
{
  return DiagnosticsVisitorRegistry<MsgT...>(std::forward<VisitorRegs>(visitor_regs)...);
}

}  // namespace ouster_ros
