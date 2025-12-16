#pragma once
#include <cstdlib>
#include <filesystem>
#include <string_view>

class AssetsManager {
public:
    /**
         * @brief 使用指定的环境变量名或备选路径初始化资产根目录。
         *
         * 根据环境变量的值（若存在且非空）或提供的后备路径确定并存储用于查找资产的根路径。
         *
         * @param env 要读取的环境变量名，用于指定资产根目录。
         * @param fallback 当环境变量未设置或为空时使用的后备文件系统路径。
         */
        explicit AssetsManager(
        std::string_view env = "TEST_ASSETS_ROOT", std::filesystem::path fallback = "/tmp/auto_aim")
        : root_ { discover_root(env, std::move(fallback)) } { }

    /**
 * @brief 获取已解析的资产根目录路径。
 *
 * @return const std::filesystem::path& 对内部存储的路径的常量引用，表示发现的资产根目录。该引用在对象存续期间有效。
 */
const std::filesystem::path& root() const noexcept { return root_; }
    /**
 * 构建并返回 assets 根目录与给定文件名拼接后的路径。
 *
 * @param filename 要追加到根目录的文件名或相对路径片段。
 * @return std::filesystem::path 拼接后的文件系统路径。
 */
std::filesystem::path path(std::string_view filename) const { return root_ / filename; }

private:
/**
 * @brief 根据环境变量或回退路径解析并返回资产根目录路径。
 *
 * 检查名为 `env` 的环境变量：若该变量存在且非空，则返回其值转为的路径，否则返回 `fallback`。
 *
 * @param env 要读取的环境变量名（例如 "TEST_ASSETS_ROOT"）。
 * @param fallback 当环境变量未设置或为空时使用的回退路径。
 * @return std::filesystem::path 环境变量值对应的路径，或 `fallback`（环境变量未设置或为空时）。
 */
static std::filesystem::path discover_root(
    std::string_view env, std::filesystem::path fallback) {
    if (const char* v = std::getenv(std::string(env).c_str()); v && *v) return std::filesystem::path { v };
    return fallback;
}

    std::filesystem::path root_;
};