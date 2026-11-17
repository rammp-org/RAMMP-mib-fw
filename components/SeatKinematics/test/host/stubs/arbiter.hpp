#pragma once

#include <cstdint>

// Host-test stand-in for the Arbiter component's arbiter.hpp, which pulls in espp headers the
// host test does not have. MIBconfig.hpp only names mib::ArbitrationMode; keep this enum in
// step with components/Arbiter/include/arbiter.hpp and add anything else it starts to use.
namespace mib {
enum class ArbitrationMode : uint8_t {
  ENFORCED,
  DISABLED,
};
} // namespace mib
