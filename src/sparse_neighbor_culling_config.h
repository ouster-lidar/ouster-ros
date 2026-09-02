/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sparse_neighbor_culling_config.h
 * @brief ROS parameter loading for multi-rule point cloud culling.
 */

#pragma once

#include "sparse_neighbor_culling_filter.h"

#include <ros/node_handle.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ouster_ros {
namespace sparse_culling_config {

inline std::runtime_error config_error(const std::string& path,
                                       const std::string& message) {
    return std::runtime_error("Invalid point filter configuration at '" + path +
                              "': " + message);
}

inline const XmlRpc::XmlRpcValue& require_member(
    const XmlRpc::XmlRpcValue& value, const std::string& member,
    const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw config_error(path, "expected a mapping");
    }
    if (!value.hasMember(member)) {
        throw config_error(path, "missing required key '" + member + "'");
    }
    return value[member];
}

inline double read_number(const XmlRpc::XmlRpcValue& value,
                          const std::string& path) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(value);
    }
    throw config_error(path, "expected a number");
}

inline int read_int(const XmlRpc::XmlRpcValue& value,
                    const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        throw config_error(path, "expected an integer");
    }
    return static_cast<int>(value);
}

inline std::string read_string(const XmlRpc::XmlRpcValue& value,
                               const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
        throw config_error(path, "expected a string");
    }
    return static_cast<std::string>(value);
}

inline void validate_allowed_members(
    const XmlRpc::XmlRpcValue& value,
    std::initializer_list<const char*> allowed,
    const std::string& path) {
    for (const auto& member : value) {
        const bool known =
            std::any_of(allowed.begin(), allowed.end(),
                        [&member](const char* name) {
                            return member.first == name;
                        });
        if (!known) {
            throw config_error(path,
                               "unknown setting '" + member.first + "'");
        }
    }
}

inline std::pair<double, double> read_number_pair(
    const XmlRpc::XmlRpcValue& value, const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        value.size() != 2) {
        throw config_error(path, "expected a two-number list");
    }
    return {read_number(value[0], path + "[0]"),
            read_number(value[1], path + "[1]")};
}

inline std::pair<int, int> read_int_pair(
    const XmlRpc::XmlRpcValue& value, const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        value.size() != 2) {
        throw config_error(path, "expected a two-integer list");
    }
    return {read_int(value[0], path + "[0]"),
            read_int(value[1], path + "[1]")};
}

inline CullingAction parse_action(const std::string& value,
                                  const std::string& path) {
    if (value == "direct") return CullingAction::REMOVE;
    if (value == "sparse") {
        return CullingAction::SPARSE_NEIGHBOR_CULL;
    }
    throw config_error(path,
                       "unknown culling method '" + value +
                           "' (expected direct or sparse)");
}

class ConditionExpressionParser {
   public:
    explicit ConditionExpressionParser(std::string expression)
        : expression_{std::move(expression)} {}

    ConditionNode parse() {
        auto condition = parse_or();
        skip_whitespace();
        if (position_ != expression_.size()) {
            fail("unexpected text");
        }
        return condition;
    }

   private:
    ConditionNode parse_or() {
        auto condition = parse_and();
        while (match("||")) {
            condition = combine(ConditionType::OR, std::move(condition),
                                parse_and());
        }
        return condition;
    }

    ConditionNode parse_and() {
        auto condition = parse_primary();
        while (match("&&")) {
            condition = combine(ConditionType::AND, std::move(condition),
                                parse_primary());
        }
        return condition;
    }

    ConditionNode parse_primary() {
        skip_whitespace();
        if (match("(")) {
            auto condition = parse_or();
            if (!match(")")) fail("expected ')'");
            return condition;
        }
        return parse_comparison();
    }

    ConditionNode parse_comparison() {
        const std::string field = parse_identifier();
        const ScalarComparison comparison = parse_operator();
        const double value = parse_number();
        ConditionNode result;
        result.type = ConditionType::COMPARISON;
        result.comparison = {field, comparison, value};
        return result;
    }

    std::string parse_identifier() {
        skip_whitespace();
        const size_t start = position_;
        if (position_ >= expression_.size() ||
            !(std::isalpha(static_cast<unsigned char>(
                  expression_[position_])) ||
              expression_[position_] == '_')) {
            fail("expected a field name");
        }
        ++position_;
        while (position_ < expression_.size() &&
               (std::isalnum(static_cast<unsigned char>(
                    expression_[position_])) ||
                expression_[position_] == '_')) {
            ++position_;
        }
        return expression_.substr(start, position_ - start);
    }

    ScalarComparison parse_operator() {
        skip_whitespace();
        if (match("<=")) return ScalarComparison::LE;
        if (match(">=")) return ScalarComparison::GE;
        if (match("==")) return ScalarComparison::EQ;
        if (match("!=")) return ScalarComparison::NE;
        if (match("<")) return ScalarComparison::LT;
        if (match(">")) return ScalarComparison::GT;
        fail("expected <, <=, >, >=, ==, or !=");
    }

    double parse_number() {
        skip_whitespace();
        size_t consumed = 0;
        double value = 0.0;
        try {
            value = std::stod(expression_.substr(position_), &consumed);
        } catch (const std::exception&) {
            fail("expected a number");
        }
        if (consumed == 0) fail("expected a number");
        position_ += consumed;
        return value;
    }

    static ConditionNode combine(ConditionType type, ConditionNode left,
                                 ConditionNode right) {
        if (left.type == type) {
            left.children.push_back(std::move(right));
            return left;
        }
        ConditionNode result;
        result.type = type;
        result.children.push_back(std::move(left));
        result.children.push_back(std::move(right));
        return result;
    }

    bool match(const std::string& token) {
        skip_whitespace();
        if (expression_.compare(position_, token.size(), token) != 0) {
            return false;
        }
        position_ += token.size();
        return true;
    }

    void skip_whitespace() {
        while (position_ < expression_.size() &&
               std::isspace(static_cast<unsigned char>(
                   expression_[position_]))) {
            ++position_;
        }
    }

    [[noreturn]] void fail(const std::string& message) const {
        throw std::runtime_error(
            "Invalid point filter expression at position " +
            std::to_string(position_) + ": " + message + " in '" +
            expression_ + "'");
    }

    std::string expression_;
    size_t position_ = 0;
};

inline ConditionNode parse_condition_expression(
    const std::string& expression) {
    return ConditionExpressionParser{expression}.parse();
}

inline ConditionNode comparison_condition(
    const std::string& field, ScalarComparison comparison, double value) {
    ConditionNode condition;
    condition.type = ConditionType::COMPARISON;
    condition.comparison = {field, comparison, value};
    return condition;
}

inline ConditionNode parse_strict_condition(
    const std::string& expression, const std::string& path) {
    ConditionNode condition;
    try {
        condition = parse_condition_expression(expression);
    } catch (const std::exception& error) {
        throw config_error(path, error.what());
    }
    if (condition.type != ConditionType::COMPARISON) {
        throw config_error(
            path, "expected one condition such as 'reflectivity <= 2'");
    }
    const auto& field = condition.comparison.field;
    if (field != "reflectivity" && field != "signal") {
        throw config_error(
            path, "field must be reflectivity or signal");
    }
    return condition;
}

inline ConditionNode parse_condition_list(
    const XmlRpc::XmlRpcValue& value, ConditionType match_type,
    const std::string& path) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        value.size() == 0) {
        throw config_error(path, "expected a non-empty condition list");
    }

    std::vector<ConditionNode> conditions;
    conditions.reserve(value.size());
    for (int index = 0; index < value.size(); ++index) {
        const std::string condition_path =
            path + "[" + std::to_string(index) + "]";
        conditions.push_back(parse_strict_condition(
            read_string(value[index], condition_path), condition_path));
    }
    if (conditions.size() == 1) return std::move(conditions.front());

    ConditionNode group;
    group.type = match_type;
    group.children = std::move(conditions);
    return group;
}

inline std::vector<FilterRule> parse_filter_rules(
    const XmlRpc::XmlRpcValue& rules_value) {
    if (rules_value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        rules_value.size() == 0) {
        throw config_error("filter_rules",
                           "expected a non-empty mapping of named rules");
    }

    std::vector<FilterRule> rules;
    rules.reserve(rules_value.size());
    for (const auto& named_rule : rules_value) {
        const std::string& name = named_rule.first;
        const auto& value = named_rule.second;
        const std::string path = "filter_rules." + name;
        if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw config_error(path, "expected a mapping");
        }
        validate_allowed_members(
            value,
            {"range_m", "culling", "any", "all", "min_neighbors",
             "kernel", "neighbor_distance_m"},
            path);
        if (name.empty()) {
            throw config_error(path, "rule name must not be empty");
        }

        FilterRule rule;
        rule.name = name;

        rule.action = parse_action(
            read_string(require_member(value, "culling", path),
                        path + ".culling"),
            path + ".culling");

        const auto range =
            read_number_pair(require_member(value, "range_m", path),
                             path + ".range_m");
        if (range.first < 0.0 || range.second <= range.first) {
            throw config_error(
                path + ".range_m",
                "minimum must be non-negative and below maximum");
        }

        const bool has_any = value.hasMember("any");
        const bool has_all = value.hasMember("all");
        if (has_any == has_all) {
            throw config_error(path,
                               "specify exactly one of any or all");
        }
        auto field_conditions = parse_condition_list(
            value[has_any ? "any" : "all"],
            has_any ? ConditionType::OR : ConditionType::AND,
            path + (has_any ? ".any" : ".all"));

        rule.condition.type = ConditionType::AND;
        rule.condition.children.push_back(comparison_condition(
            "range", ScalarComparison::GE, range.first));
        rule.condition.children.push_back(comparison_condition(
            "range", ScalarComparison::LT, range.second));
        rule.condition.children.push_back(
            std::move(field_conditions));

        if (rule.action == CullingAction::SPARSE_NEIGHBOR_CULL) {
            rule.sparse.min_neighbors = read_int(
                require_member(value, "min_neighbors", path),
                path + ".min_neighbors");
            const auto kernel =
                read_int_pair(require_member(value, "kernel", path),
                              path + ".kernel");
            rule.sparse.kernel_height = kernel.first;
            rule.sparse.kernel_width = kernel.second;
            rule.sparse.neighbor_distance_m = read_number(
                require_member(value, "neighbor_distance_m", path),
                path + ".neighbor_distance_m");
        } else if (value.hasMember("min_neighbors") ||
                   value.hasMember("kernel") ||
                   value.hasMember("neighbor_distance_m")) {
            throw config_error(
                path,
                "neighbor settings can only be used with culling: sparse");
        }
        rules.push_back(std::move(rule));
    }
    return rules;
}

inline SparseNeighborCullingConfig disabled_config() {
    SparseNeighborCullingConfig config;
    config.enabled = false;
    return config;
}

inline SparseNeighborCullingConfig parse_config(
    const XmlRpc::XmlRpcValue& value) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw config_error("point_filter_config", "expected a mapping");
    }
    validate_allowed_members(value, {"filter_rules"},
                             "point_filter_config");

    SparseNeighborCullingConfig config;
    config.enabled = true;
    config.keep_organized = true;
    config.rules = parse_filter_rules(require_member(
        value, "filter_rules", "point_filter_config"));
    return config;
}

inline SparseNeighborCullingConfig load_config(
    const ros::NodeHandle& private_node_handle) {
    XmlRpc::XmlRpcValue config;
    if (!private_node_handle.getParam("point_filter_config", config)) {
        return disabled_config();
    }
    return parse_config(config);
}

}  // namespace sparse_culling_config
}  // namespace ouster_ros
