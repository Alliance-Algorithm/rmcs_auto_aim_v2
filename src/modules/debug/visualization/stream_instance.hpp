#pragma once
#include "stream_context.hpp"
#include <memory>

namespace rmcs::debug {

class StreamSession {
public:
    using StreamTarget = StreamContext::StreamTarget;
    using StreamType   = StreamContext::StreamType;
    using VideoFormat  = StreamContext::VideoFormat;

public:
    explicit StreamSession(StreamType, const StreamTarget&, const VideoFormat&) noexcept;
    ~StreamSession() noexcept;

    StreamSession(const StreamSession&)            = delete;
    StreamSession& operator=(const StreamSession&) = delete;

    StreamSession(StreamSession&&) noexcept            = default;
    StreamSession& operator=(StreamSession&&) noexcept = default;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
