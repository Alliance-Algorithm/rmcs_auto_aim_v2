#pragma once
#include <string>

namespace rmcs::module {

struct UdpConfig {
    int w;
    int h;
    int hz;
    std::string host;
    std::string port;
};
struct TcpConfig {
    int w;
    int h;
    int hz;
};

}