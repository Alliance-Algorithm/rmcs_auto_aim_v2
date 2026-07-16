#pragma once
#include <array>
#include <cstdint>
#include <generator>
#include <utility>

namespace rmcs {

namespace id::details {
    constexpr auto id_underlyings = std::array {
        std::uint16_t { 0 << 0 },
        std::uint16_t { 1 << 0 },
        std::uint16_t { 1 << 1 },
        std::uint16_t { 1 << 2 },
        std::uint16_t { 1 << 3 },
        std::uint16_t { 1 << 4 },
        std::uint16_t { 1 << 5 },
        std::uint16_t { 1 << 6 },
        std::uint16_t { 1 << 7 },
        std::uint16_t { 1 << 8 },
        std::uint16_t { 1 << 9 },
        std::uint16_t { 1 << 10 },
        std::uint16_t { 1 << 11 },
        std::uint16_t { 1 << 12 },
    };
}
enum class DeviceId : std::uint16_t {
    UNKNOWN    = id::details::id_underlyings[0],
    HERO       = id::details::id_underlyings[1],
    ENGINEER   = id::details::id_underlyings[2],
    INFANTRY_3 = id::details::id_underlyings[3],
    INFANTRY_4 = id::details::id_underlyings[4],
    INFANTRY_5 = id::details::id_underlyings[5],
    AERIAL     = id::details::id_underlyings[6],
    SENTRY     = id::details::id_underlyings[7],
    DART       = id::details::id_underlyings[8],
    RADAR      = id::details::id_underlyings[9],
    OUTPOST    = id::details::id_underlyings[10],
    BASE       = id::details::id_underlyings[11],
    RUNE       = id::details::id_underlyings[12],
};
constexpr auto to_index(DeviceId id) noexcept -> std::size_t {
    switch (id) {
        // clang-format off
        case DeviceId::UNKNOWN:    return 0;
        case DeviceId::HERO:       return 1;
        case DeviceId::ENGINEER:   return 2;
        case DeviceId::INFANTRY_3: return 3;
        case DeviceId::INFANTRY_4: return 4;
        case DeviceId::INFANTRY_5: return 5;
        case DeviceId::AERIAL:     return 6;
        case DeviceId::SENTRY:     return 7;
        case DeviceId::DART:       return 8;
        case DeviceId::RADAR:      return 9;
        case DeviceId::OUTPOST:    return 10;
        case DeviceId::BASE:       return 11;
        case DeviceId::RUNE:       return 12;
        // clang-format on
    }
    return { };
}
constexpr auto to_string(DeviceId id) noexcept {
    switch (id) {
        // clang-format off
        case DeviceId::UNKNOWN:    return "UNKNOWN";
        case DeviceId::HERO:       return "HERO";
        case DeviceId::ENGINEER:   return "ENGINEER";
        case DeviceId::INFANTRY_3: return "INFANTRY_3";
        case DeviceId::INFANTRY_4: return "INFANTRY_4";
        case DeviceId::INFANTRY_5: return "INFANTRY_5";
        case DeviceId::AERIAL:     return "AERIAL";
        case DeviceId::SENTRY:     return "SENTRY";
        case DeviceId::DART:       return "DART";
        case DeviceId::RADAR:      return "RADAR";
        case DeviceId::OUTPOST:    return "OUTPOST";
        case DeviceId::BASE:       return "BASE";
        case DeviceId::RUNE:       return "RUNE";
        // clang-format on
    }
    return "UNREACHABLE";
}
constexpr auto from_index(std::size_t data) noexcept -> DeviceId {
    return (data < id::details::id_underlyings.size())
        ? DeviceId { id::details::id_underlyings[data] }
        : DeviceId::UNKNOWN;
}

struct DeviceIds {
    uint16_t data = std::to_underlying(DeviceId::UNKNOWN);

    constexpr DeviceIds()                            = default;
    constexpr DeviceIds(const DeviceIds&)            = default;
    constexpr DeviceIds& operator=(const DeviceIds&) = default;

    constexpr explicit DeviceIds(uint16_t data) noexcept
        : data { data } { };

    template <std::same_as<DeviceId>... Ids>
    constexpr explicit DeviceIds(Ids... ids)
        : data { static_cast<uint16_t>((std::to_underlying(ids) | ...)) } { }

    constexpr auto operator==(const DeviceIds& o) const noexcept -> bool = default;

    constexpr auto operator|(const DeviceIds& other) const {
        return DeviceIds { static_cast<uint16_t>(data | other.data) };
    }
    constexpr auto operator&(const DeviceIds& other) const {
        return DeviceIds { static_cast<uint16_t>(data & other.data) };
    }

    constexpr auto contains(DeviceId id) const noexcept -> bool {
        return 0 != (data & std::to_underlying(id));
    }
    constexpr auto empty() const noexcept -> bool {
        return std::to_underlying(DeviceId::UNKNOWN) == data;
    }

    constexpr auto append(DeviceId id) noexcept -> void { data |= std::to_underlying(id); }
    constexpr auto remove(DeviceId id) noexcept -> void { data &= ~std::to_underlying(id); }

    constexpr auto length() const noexcept -> std::size_t { return std::popcount(data); }

    auto items() const -> std::generator<DeviceId> {
        using namespace id::details;
        for (const auto id : id_underlyings) {
            const auto device_id = static_cast<DeviceId>(id);
            if (contains(device_id)) {
                co_yield device_id;
            }
        }
    }

    static constexpr auto None() { return DeviceIds { }; }
    static constexpr auto Full() { return DeviceIds { (1 << 12) - 1 }; }

    static constexpr auto kLargeArmor() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::ENGINEER,
        };
    }
    static constexpr auto kSmallArmor() {
        return DeviceIds {
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
            DeviceId::OUTPOST,
            DeviceId::BASE,
        };
    }
    static constexpr auto kGround() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::ENGINEER,
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
        };
    }
    static constexpr auto kOffensive() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
        };
    }
    static constexpr auto kBuilding() {
        return DeviceIds {
            DeviceId::OUTPOST,
            DeviceId::BASE,
            DeviceId::RUNE,
        };
    }
    static constexpr auto kInfantry() {
        return DeviceIds {
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
        };
    }
};
static_assert((DeviceIds::kSmallArmor() & DeviceIds::kLargeArmor()) == DeviceIds::None());
}
