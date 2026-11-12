#include "utils/licenses.hpp"

// Defined in the generated license_texts.cpp (see CMakeLists.txt).
extern const unsigned char dsu_license_gpl[];
extern const unsigned char dsu_license_third_party[];

namespace Licenses {

const char* project_notice() {
    return
        "PSMove-DSU - a DSU (Cemuhook) server for PlayStation Move controllers\n"
        "Copyright (C) 2025-2026 Swordmaster3214 and contributors\n"
        "https://github.com/Swordmaster3214/PSMove-DSU\n"
        "\n"
        "This program is free software: you can redistribute it and/or modify it\n"
        "under the terms of the GNU General Public License, version 3, as\n"
        "published by the Free Software Foundation.\n"
        "\n"
        "This program is distributed in the hope that it will be useful, but\n"
        "WITHOUT ANY WARRANTY; without even the implied warranty of\n"
        "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General\n"
        "Public License for more details.\n"
        "\n"
        "The source code is available at the address above.\n";
}

const char* gpl_text() { return reinterpret_cast<const char*>(dsu_license_gpl); }

const char* third_party_text() { return reinterpret_cast<const char*>(dsu_license_third_party); }

void print_all(std::ostream& out) {
    const char* rule = "\n================================================================================\n\n";
    out << project_notice() << rule << third_party_text() << rule << gpl_text();
}

} // namespace Licenses
