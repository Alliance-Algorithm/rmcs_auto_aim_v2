#include "identifier.hpp"

using namespace rmcs::kernal;
using namespace identifier::details;

struct IdentifierImpl::Impl {
    CampColor color  = CampColor::UNKNOWN;
    DeviceIds target = DeviceIds::Full();
};

IdentifierImpl::IdentifierImpl() noexcept
    : pimpl { std::make_unique<Impl>() } { }

IdentifierImpl::~IdentifierImpl() noexcept = default;
