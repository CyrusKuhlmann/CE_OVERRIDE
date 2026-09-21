#pragma once

#include "command/command.h"

#include "pros/llemu.hpp"

#include <cstddef>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// Prints one brain LCD line when the command starts. {} placeholders are filled
// from values or from zero-argument functions, which are called at that moment.
class Print : public Command {
public:
    Print(int line, std::function<std::string()> message) : line_(line), message_(std::move(message)) {}

    template <typename... Args>
    Print(int line, std::string format, Args&&... args)
        : Print(line, bind(std::move(format), std::forward<Args>(args)...)) {}

    void initialize() override { pros::lcd::set_text(static_cast<std::int16_t>(line_), message_()); }

    bool isFinished() override { return true; }

    std::vector<Subsystem*> getRequirements() override { return {}; }

private:
    template <typename T>
    static constexpr bool kUnsupported = false;

    template <typename T>
    static std::string stringify(const T& value) {
        if constexpr (std::is_invocable_v<const T&>) {
            using Result = std::invoke_result_t<const T&>;
            static_assert(!std::is_void_v<Result>, "Print function must return a value");
            return stringify(std::invoke(value));
        } else if constexpr (std::is_convertible_v<const T&, std::string>) {
            return std::string(value);
        } else if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
            return std::to_string(value);
        } else {
            static_assert(kUnsupported<T>, "Print values must be numbers, strings, or functions that return one");
            return {};
        }
    }

    template <typename... Args>
    static std::function<std::string()> bind(std::string format, Args&&... args) {
        std::vector<std::function<std::string()>> parts;
        parts.reserve(sizeof...(Args));
        if constexpr (sizeof...(Args) > 0) {
            (parts.push_back([arg = std::decay_t<Args>(std::forward<Args>(args))]() { return stringify(arg); }), ...);
        }

        return [format = std::move(format), parts = std::move(parts)] {
            std::string out;
            std::size_t arg = 0;
            for (std::size_t i = 0; i < format.size(); ++i) {
                if (format[i] == '{' && i + 1 < format.size() && format[i + 1] == '}' && arg < parts.size()) {
                    out += parts[arg++]();
                    ++i;
                } else {
                    out.push_back(format[i]);
                }
            }
            return out;
        };
    }

    int line_;
    std::function<std::string()> message_;
};
