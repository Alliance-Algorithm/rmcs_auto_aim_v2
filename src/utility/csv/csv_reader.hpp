#pragma once

#include <charconv>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <ranges>

#include "utility/serializable.hpp"
#include "utility/string.hpp"

namespace rmcs::util {

class CsvReader {
public:
    explicit CsvReader(const std::filesystem::path& path) {
        ifstream_.imbue(std::locale::classic());
        ifstream_.open(path);
        if (!ifstream_.is_open())
            throw std::runtime_error("Failed to open CSV file: " + path.string());

        if (!read_line())
            throw std::runtime_error(
                "Failed to read the header line of CSV file: " + path.string());
        split_line();
        parse_header();

        next();
    }

    void reset() {
        ifstream_.clear();
        ifstream_.seekg(0);
        eof_            = false;
        corrupted_line_ = false;
        line_buffer_.clear();
        line_fields_.clear();

        if (!read_line()) throw std::runtime_error("Failed to read the header line of CSV file");
        next();
    }

    bool next() {
        if (!read_line()) return false;

        split_line();
        if (line_fields_.size() != header_map_.size()) {
            line_fields_.clear();
            corrupted_line_ = true;
            return false;
        }

        corrupted_line_ = false;
        return true;
    }

    [[nodiscard]] bool eof() const { return eof_; }
    [[nodiscard]] bool corrupted_line() const { return corrupted_line_; }

    std::expected<std::string_view, std::string> field_at(const std::string& name) const {
        if (line_fields_.empty())
            return std::unexpected(
                std::format("EOF or corrupted line: {}, {}", eof(), corrupted_line()));

        const auto it = header_map_.find(name);
        if (it == header_map_.cend())
            return std::unexpected(std::format("Header '{}' not found", name));

        return line_fields_.at(it->second);
    }

private:
    [[nodiscard]] bool read_line() {
        line_buffer_.clear();
        line_fields_.clear();
        if (!std::getline(ifstream_, line_buffer_)) {
            eof_ = true;
            return false;
        }
        eof_ = false;
        return true;
    }

    void split_line() {
        line_fields_.clear();
        std::ranges::copy(std::views::split(line_buffer_, ',') | std::views::transform([](auto r) {
            return trim(std::string_view(r.begin(), r.end()));
        }),
            std::back_inserter(line_fields_));
    }

    void parse_header() {
        header_map_.clear();
        for (size_t i = 0; i < line_fields_.size(); i++) {
            const auto [_, inserted] = header_map_.insert({ std::string { line_fields_[i] }, i });
            if (!inserted)
                throw std::runtime_error(std::format("Duplicate CSV header '{}'", line_fields_[i]));
        }
    }

    std::ifstream ifstream_;

    std::string line_buffer_;
    std::vector<std::string_view> line_fields_;
    bool eof_ = false, corrupted_line_ = false;

    std::map<std::string, size_t> header_map_;
};

template <>
struct SerializableSource<CsvReader> {
    static auto get(const CsvReader& source, const std::string& name, bool& target) noexcept
        -> std::expected<void, std::string> {
        auto field = source.field_at(name);
        if (!field) return std::unexpected { std::move(field.error()) };
        const auto& view = field.value();

        if (view == "1") { // NOLINT(bugprone-branch-clone)
            target = true;
            return { };
        } else if (view == "0") { // NOLINT(bugprone-branch-clone)
            target = false;
            return { };
        } else if (ascii_iequals(view, "true") || ascii_iequals(view, "on")) {
            target = true;
            return { };
        } else if (ascii_iequals(view, "false") || ascii_iequals(view, "off")) {
            target = false;
            return { };
        }

        return std::unexpected { std::format("Invalid bool value: {}", view) };
    }

    template <typename T>
        requires(std::is_integral_v<T> || std::is_floating_point_v<T>)
    static auto get(const CsvReader& source, const std::string& name, T& target) noexcept
        -> std::expected<void, std::string> {
        auto field = source.field_at(name);
        if (!field) return std::unexpected { std::move(field.error()) };
        const auto& view = field.value();

        const auto result = std::from_chars(view.data(), view.data() + view.size(), target);
        if (result.ec == std::errc { } && result.ptr == view.data() + view.size()) return { };

        return std::unexpected { std::format("Invalid {} value: {}", typeid(T).name(), view) };
    }
};

}
