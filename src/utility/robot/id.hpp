#pragma once
#include <array>
#include <cstdint>
#include <string_view>
#include <utility>

namespace rmcs {

namespace id::details {
    constexpr auto id_underlyings = std::array {
        uint16_t { 0 << 0 },
        uint16_t { 1 << 0 },
        uint16_t { 1 << 1 },
        uint16_t { 1 << 2 },
        uint16_t { 1 << 3 },
        uint16_t { 1 << 4 },
        uint16_t { 1 << 5 },
        uint16_t { 1 << 6 },
        uint16_t { 1 << 7 },
        uint16_t { 1 << 8 },
        uint16_t { 1 << 9 },
        uint16_t { 1 << 10 },
    };
}
enum class DeviceId : uint16_t {
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
        // clang-format on
    }
    return {};
}
constexpr auto to_string(DeviceId id) noexcept -> std::string_view {
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
        // clang-format on
    }
    return {};
}
constexpr auto from_index(std::size_t data) noexcept -> DeviceId {
    return (data < id::details::id_underlyings.size())
        ? DeviceId { id::details::id_underlyings[data] }
        : DeviceId::UNKNOWN;
}

struct DeviceIds {
    uint16_t data = std::to_underlying(DeviceId::UNKNOWN);

    /**
 * @brief 创建表示无设备的 DeviceIds 实例。
 *
 * @return DeviceIds 其内部位掩码等于 `DeviceId::UNKNOWN` 的底层值（表示空集合）。
 */
static constexpr auto None() { return DeviceIds {}; }
    /**
 * @brief 构造一个包含所有已定义设备 ID 的 DeviceIds 实例。
 *
 * @return DeviceIds 包含所有 11 个设备 ID 的位掩码（内部数据的第 0 到 10 位都被置为 1）。
 */
static constexpr auto Full() { return DeviceIds { (1 << 11) - 1 }; }

    /**
 * @brief 构造一个空的 DeviceIds 实例，data 初始化为 `DeviceId::UNKNOWN` 的底层值。
 */
constexpr DeviceIds()                            = default;
    /**
 * @brief 构造一个与给定 DeviceIds 值相同的副本。
 *
 * @param other 要复制的 DeviceIds 对象。
 */
constexpr DeviceIds(const DeviceIds&)            = default;
    /**
 * @brief 将另一个 DeviceIds 的位集合拷贝到当前对象。
 *
 * @param other 要复制的 DeviceIds 实例。
 * @return DeviceIds& 引用到赋值后的当前对象（`*this`）。
 */
constexpr DeviceIds& operator=(const DeviceIds&) = default;

    /**
         * @brief 使用给定的原始位掩码创建一个 DeviceIds 实例。
         *
         * 将参数 `data` 作为内部位掩码直接存储，位含义按 DeviceId 的底层编码映射（每个设备对应各自的位）。
         *
         * @param data 用作内部位掩码的原始 `uint16_t` 值；各位表示相应的 `DeviceId` 是否存在。
         */
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

    constexpr static auto kLargeArmorDevices() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::ENGINEER,
            DeviceId::BASE,
        };
    }
    constexpr static auto kSmallArmorDevices() {
        return DeviceIds {
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
            DeviceId::OUTPOST,
        };
    }
    constexpr static auto kGroundDevices() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::ENGINEER,
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
        };
    }
    constexpr static auto kOffensiveDevices() {
        return DeviceIds {
            DeviceId::HERO,
            DeviceId::INFANTRY_3,
            DeviceId::INFANTRY_4,
            DeviceId::INFANTRY_5,
            DeviceId::SENTRY,
        };
    }
    constexpr static auto kBuildingDevices() {
        return DeviceIds {
            DeviceId::OUTPOST,
            DeviceId::BASE,
        };
    }
};

}