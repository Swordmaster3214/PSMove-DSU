#!/usr/bin/env bash
# Shared CI steps for the built executable, used by every build workflow so
# that all platforms are checked the same way. Runs under bash on Linux,
# macOS and Windows (Git Bash on GitHub's Windows runners).
#
#   ci_binary.sh locate             Print the path of the built executable
#   ci_binary.sh inspect <exe>      Check architecture and runtime dependencies
#   ci_binary.sh smoke <exe>        Run the server with virtual controllers and
#                                   check it with test/dsu_protocol_check.py
#   ci_binary.sh gui <exe>          Linux only: check the GUI opens a window
#                                   (needs xvfb-run)
#
# Exit code is non-zero on failure; problems are also reported as GitHub
# ::error:: annotations.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP="${RUNNER_TEMP:-${TMPDIR:-/tmp}}"

case "$(uname -s)" in
    Darwin) OS=macos ;;
    Linux) OS=linux ;;
    MINGW*|MSYS*|CYGWIN*) OS=windows ;;
    *) OS=unknown ;;
esac

fail() {
    echo "::error::$*"
    exit 1
}

section() {
    echo
    echo "== $*"
}

cmd_locate() {
    local exe candidates
    # The executable lands in build/ (CMakeLists.txt sets the output directory),
    # or build/Release/ with a multi-config generator that ignores it. build/ also
    # holds same-named Visual Studio project files, so match .exe / executables only.
    if [ "$OS" = windows ]; then
        candidates="build/dsu_server_psmove_*.exe build/Release/dsu_server_psmove_*.exe"
    else
        candidates="build/dsu_server_psmove_* build/Release/dsu_server_psmove_*"
    fi
    for exe in $candidates; do
        if [ -f "$exe" ] && { [ "$OS" = windows ] || [ -x "$exe" ]; }; then
            echo "$exe"
            return 0
        fi
    done
    fail "No dsu_server_psmove_* executable found under build/"
}

inspect_macos() {
    local exe="$1" expected archs non_system
    expected="$(uname -m)"
    archs="$(lipo -archs "$exe")"
    echo "Architectures: $archs (runner: $expected)"
    [ "$archs" = "$expected" ] || fail "Expected a $expected binary, got: $archs"

    section "Minimum macOS version"
    otool -l "$exe" | grep -A4 LC_BUILD_VERSION | grep -E "minos|sdk"

    section "Linked libraries (must all be part of macOS)"
    otool -L "$exe"
    non_system="$(otool -L "$exe" | tail -n +2 | awk '{print $1}' | grep -v -E '^(/usr/lib/|/System/Library/)' || true)"
    [ -z "$non_system" ] || fail "Depends on libraries users won't have: $non_system"
}

inspect_linux() {
    local exe="$1" desc expected bad
    desc="$(file -b "$exe")"
    case "$(uname -m)" in
        x86_64) expected="x86-64" ;;
        aarch64) expected="aarch64" ;;
        *) expected="$(uname -m)" ;;
    esac
    echo "$desc"
    case "$desc" in
        *"$expected"*) ;;
        *) fail "Expected a $expected binary" ;;
    esac

    section "Direct library dependencies"
    readelf -d "$exe" | grep NEEDED
    # libstdc++/libgcc are linked statically on purpose (see CMakeLists.txt), and
    # GLFW loads X11/Wayland/GL at runtime so the server still starts without them.
    # (Checked on NEEDED entries: ldd also lists what system libraries pull in.)
    bad="$(readelf -d "$exe" | grep NEEDED | grep -E -o '(libstdc\+\+|libgcc_s|libX11|libwayland-client|libGL)[^]]*' || true)"
    [ -z "$bad" ] || fail "Unexpected dynamic dependency: $bad"

    section "Full runtime library tree (ldd)"
    ldd "$exe"

    section "Newest glibc symbol version required (the binary needs at least this glibc)"
    objdump -T "$exe" | grep -o 'GLIBC_[0-9.]*' | sort -u -V | tail -n 1
}

inspect_windows() {
    local exe="$1" vswhere dumpbin deps machine
    vswhere="/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
    [ -f "$vswhere" ] || fail "vswhere.exe not found; can't locate dumpbin"
    dumpbin="$("$vswhere" -latest -products '*' -find 'VC/Tools/MSVC/**/bin/Hostx64/x64/dumpbin.exe' | head -n 1 | tr -d '\r')"
    [ -n "$dumpbin" ] || fail "dumpbin.exe not found"

    machine="$("$dumpbin" //headers "$exe" | grep -i 'machine (' | head -n 1)"
    echo "Machine: $machine"
    case "$(uname -m)" in
        x86_64) echo "$machine" | grep -q 'x64' || fail "Expected an x64 binary" ;;
    esac

    section "DLL dependencies"
    deps="$("$dumpbin" //dependents "$exe" | grep -i '\.dll' | sed 's/^ *//' | tr -d '\r')"
    echo "$deps"
    # The MSVC runtime is linked statically so users don't need the VC++ Redistributable.
    if echo "$deps" | grep -qiE '^(vcruntime|msvcp|ucrtbase)'; then
        fail "Depends on the Visual C++ runtime DLLs; it should be linked statically"
    fi
}

cmd_inspect() {
    local exe="${1:?usage: ci_binary.sh inspect <exe>}"
    [ -f "$exe" ] || fail "$exe not found"
    section "$(basename "$exe") ($(wc -c < "$exe" | tr -d ' ') bytes)"
    case "$OS" in
        macos) inspect_macos "$exe" ;;
        linux) inspect_linux "$exe" ;;
        windows) inspect_windows "$exe" ;;
        *) fail "Unsupported OS: $(uname -s)" ;;
    esac
    echo
    echo "Inspection passed."
}

python_cmd() {
    # On Windows "python3" may be the Microsoft Store stub; prefer "python" there.
    if [ "$OS" = windows ]; then
        command -v python || command -v python3
    else
        command -v python3 || command -v python
    fi
}

stop_server() {
    local pid="$1" winpid
    if [ "$OS" = windows ]; then
        # Git Bash can't deliver Ctrl+C to a native Windows process.
        winpid="$(cat "/proc/$pid/winpid" 2>/dev/null || true)"
        if [ -n "$winpid" ]; then
            taskkill //F //PID "$winpid" > /dev/null 2>&1
        else
            kill "$pid" 2> /dev/null
        fi
        wait "$pid" 2> /dev/null
        return 0
    fi
    kill -INT "$pid" 2> /dev/null
    for _ in $(seq 1 50); do
        kill -0 "$pid" 2> /dev/null || break
        sleep 0.1
    done
    if kill -0 "$pid" 2> /dev/null; then
        kill -KILL "$pid" 2> /dev/null
        wait "$pid" 2> /dev/null
        echo "::error::Server did not shut down within 5 s of Ctrl+C"
        return 1
    fi
    wait "$pid"
    local code=$?
    [ "$code" -eq 0 ] || { echo "::error::Server exited with code $code after Ctrl+C"; return 1; }
    return 0
}

cmd_smoke() {
    local exe="${1:?usage: ci_binary.sh smoke <exe>}" py log pid result stopped
    py="$(python_cmd)" || fail "Python 3 not found"
    log="$TMP/dsu-smoke-server.log"

    section "$exe --help"
    "$exe" --help || fail "--help failed"

    section "Protocol check against: $exe --headless --simulate 2"
    "$exe" --headless --simulate 2 --config "$TMP/dsu-smoke-config.ini" > "$log" 2>&1 &
    pid=$!
    "$py" "$REPO_ROOT/test/dsu_protocol_check.py"
    result=$?
    stop_server "$pid"
    stopped=$?

    section "Server log"
    cat "$log"

    [ "$result" -eq 0 ] || fail "DSU protocol check failed"
    [ "$stopped" -eq 0 ] || fail "Server did not shut down cleanly"
    echo
    echo "Smoke test passed."
}

cmd_gui() {
    local exe="${1:?usage: ci_binary.sh gui <exe>}" log code
    [ "$OS" = linux ] || fail "The GUI check needs xvfb-run (Linux only)"
    command -v xvfb-run > /dev/null || fail "xvfb-run not installed"
    log="$TMP/dsu-gui.log"

    section "GUI under a virtual X server"
    # Ctrl+C after 6 s; the GUI should be running until then and exit cleanly.
    xvfb-run -a -s "-screen 0 1280x800x24" \
        timeout -s INT 6 "$exe" --verbose --simulate 1 --config "$TMP/dsu-gui-config.ini" > "$log" 2>&1
    code=$?
    cat "$log"

    if grep -q "Could not open a window" "$log"; then
        fail "The GUI could not open a window"
    fi
    # 124 = timeout fired (expected); 0 = exited cleanly on the signal
    [ "$code" -eq 0 ] || [ "$code" -eq 124 ] || fail "GUI run exited with code $code"
    echo
    echo "GUI check passed."
}

command="${1:-}"
shift || true
case "$command" in
    locate) cmd_locate "$@" ;;
    inspect) cmd_inspect "$@" ;;
    smoke) cmd_smoke "$@" ;;
    gui) cmd_gui "$@" ;;
    *)
        echo "usage: ci_binary.sh locate | inspect <exe> | smoke <exe> | gui <exe>" >&2
        exit 2
        ;;
esac
