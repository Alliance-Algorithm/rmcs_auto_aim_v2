#pragma once
#include "stream_context.hpp"
#include <functional>

namespace rmcs::debug {

using StreamTarget = StreamContext::StreamTarget;
using StreamType   = StreamContext::StreamType;
using VideoFormat  = StreamContext::VideoFormat;
using FrameRef     = StreamContext::FrameRef;

class StreamSession {
public:
    struct Target {
        StreamTarget target;
        StreamType type;
        VideoFormat format;
    };

public:
    explicit StreamSession(StreamType, const StreamTarget&, const VideoFormat&) noexcept;
    explicit StreamSession(const Target&) noexcept;

    ~StreamSession() noexcept;

    StreamSession(const StreamSession&)            = delete;
    StreamSession& operator=(const StreamSession&) = delete;

    StreamSession(StreamSession&&) noexcept            = default;
    StreamSession& operator=(StreamSession&&) noexcept = default;

    auto set_notifier(std::function<void(const std::string&)>) noexcept -> void;
    auto open() noexcept -> std::expected<void, std::string_view>;

    auto push_frame(FrameRef) noexcept -> bool;

    auto session_description_protocol() const noexcept -> std::expected<std::string, std::string>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
