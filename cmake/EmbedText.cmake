# License texts built into the executable (--licenses and the GUI's About tab),
# so the notices travel with the binary. Stored as byte arrays rather than
# string literals, which MSVC caps at 64 KB.
function(dsu_embed_text out_var symbol file)
    file(READ "${file}" hex HEX)
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," hex "${hex}")
    string(REPEAT "0x..," 24 row)  # CMake regexes have no {n}
    string(REGEX REPLACE "(${row})" "\\1\n" hex "${hex}")
    # unsigned: the texts are UTF-8, and bytes >= 0x80 don't fit a signed char.
    set(${out_var} "${${out_var}}extern const unsigned char ${symbol}[] = {\n${hex}0x00};\n" PARENT_SCOPE)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${file}")
endfunction()
