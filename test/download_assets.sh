#!/bin/bash

# 设置：如果任何命令失败，立即退出，对管道也生效
set -eo pipefail

# --- 配置 ---
#  获取脚本自身的目录，用于定位配置文件
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
CONFIG_YML="$SCRIPT_DIR/asset.yml"
# 资源存放目录，优先使用 TEST_ASSETS_ROOT，默认为 /tmp/auto_aim（与测试代码一致）
DOWNLOAD_DIR="${TEST_ASSETS_ROOT:-/tmp/auto_aim}"
# 初始化为空字符串，表示当前没有文件在下载**
CURRENT_DOWNLOAD_PATH=""
# -----------------

# cleanup 捕获中断信号，删除由 CURRENT_DOWNLOAD_PATH 指向的任何不完整下载文件并以状态 1 退出。
cleanup() {
    echo "" 
    echo "[INTERRUPT] 捕获到 Ctrl+C。开始清理..." >&2

    if [ -n "$CURRENT_DOWNLOAD_PATH" ]; then
        if [ -f "$CURRENT_DOWNLOAD_PATH" ]; then
            echo "[CLEANUP] 删除不完整的文件: $CURRENT_DOWNLOAD_PATH" >&2
            rm -f "$CURRENT_DOWNLOAD_PATH"
        fi
    fi
    echo "[EXIT] 脚本已终止。" >&2
    exit 1
}

trap cleanup INT TERM

# check_dependencies 验证系统中是否安装了 yq 和 curl；如果任一缺失则输出错误信息并以状态码 1 退出。
check_dependencies() {
    if ! command -v yq &> /dev/null || ! command -v curl &> /dev/null; then
        echo "[FATAL] 错误: 缺少必要的依赖工具 (yq 或 curl)。" >&2
        echo "请安装 yq (用于解析 YAML) 和 curl (用于下载)。" >&2
        exit 1
    fi
}

# download_asset 将指定 asset_id 对应的 URL 下载到 $DOWNLOAD_DIR（若已存在则跳过），下载失败时移除部分文件并设置非零退出状态。
download_asset() {
    local asset_id="$1"
    local url="$2"
    local filename
    filename=$(basename "$url")
    local local_path="$DOWNLOAD_DIR/$filename"
    
    if [ -f "$local_path" ]; then
        echo "  [SKIP] $asset_id 已存在。"
        return 0
    fi
    
    echo "  [INFO] 正在下载 $asset_id -> $filename..."
 
    CURRENT_DOWNLOAD_PATH="$local_path"

    if curl -fsSL -o "$local_path" "$url" &> /dev/null; then
        echo "  [SUCCESS] -> $local_path"
        CURRENT_DOWNLOAD_PATH=""
        return 0
    else
        echo "  [ERROR] 下载失败: $asset_id (URL: $url)" >&2
        rm -f "$local_path"
        CURRENT_DOWNLOAD_PATH=""
        return 1
    fi
}

# main 负责读取配置文件中的资源映射并逐一调用 download_asset 下载每个资源，处理依赖检查、缺失配置、空资源集、下载失败计数以及中断时的清理和退出码。
main() {
    local RESOURCE_MAPPING
    local FAILED_COUNT=0
    
    check_dependencies
    
    echo "--- 资源下载脚本启动 ---"
    
    if [ ! -f "$CONFIG_YML" ]; then
        echo "[FATAL] 配置文件未找到于 $CONFIG_YML" >&2
        trap - INT TERM
        exit 1
    fi
    
    mkdir -p "$DOWNLOAD_DIR"

    RESOURCE_MAPPING=$(yq '.resources | to_entries | .[] | "\(.key)|\(.value)"' "$CONFIG_YML")

    if [ -z "$RESOURCE_MAPPING" ]; then
        echo "[WARNING] YAML 文件中未定义任何 'resources'，退出。"
        trap - INT TERM
        exit 0
    fi

    echo "找到 $(echo "$RESOURCE_MAPPING" | wc -l | tr -d '[:space:]') 个资源需要处理。"

    while IFS='|' read -r asset_id url; do
        asset_id=$(echo "$asset_id" | tr -d '"') 
        url=$(echo "$url" | tr -d '"')
        
        download_asset "$asset_id" "$url" || FAILED_COUNT=$((FAILED_COUNT + 1))
        
    done <<< "$RESOURCE_MAPPING"

    echo ""
    echo "--- 资源下载完成 ---"
    
    trap - INT TERM
    
    if [ "$FAILED_COUNT" -eq 0 ]; then
        echo "[INFO] 所有资源下载成功或已跳过。"
        exit 0
    else
        echo "[ERROR] 警告：有 $FAILED_COUNT 个资源下载失败。" >&2
        exit 1
    fi
}

main