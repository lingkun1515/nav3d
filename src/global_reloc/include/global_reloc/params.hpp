#pragma once

#include "global_reloc/types.hpp"
#include <string>

namespace global_reloc {

/// Load parameters from a YAML-like file (subset: "key: value", nested by 2-space indent).
/// Throws std::runtime_error on parse failure.
RelocParams loadParamsFromFile(const std::string& path);

/// Load parameters from an inline YAML string (same subset). For tests.
RelocParams loadParamsFromString(const std::string& yaml);

}  // namespace global_reloc
