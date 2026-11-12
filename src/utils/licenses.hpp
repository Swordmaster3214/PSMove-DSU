#pragma once
#include <ostream>

// License texts built into the executable, so the notices required by the
// bundled third-party code always ship with it.
namespace Licenses {
    // Short copyright / no-warranty notice for PSMove-DSU itself.
    const char* project_notice();
    // Full text of the GNU GPL v3 (the LICENSE file).
    const char* gpl_text();
    // THIRD_PARTY_LICENSES.md.
    const char* third_party_text();
    // Everything above, for --licenses.
    void print_all(std::ostream& out);
}
