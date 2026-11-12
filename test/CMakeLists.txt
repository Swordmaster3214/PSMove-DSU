# Added from the root CMakeLists.txt when -DDSU_BUILD_TESTS=ON.

find_package(Python3 COMPONENTS Interpreter)

if(Python3_Interpreter_FOUND)
    # Starts the freshly built server with two simulated controllers, runs
    # dsu_protocol_check.py against it, then shuts the server down.
    add_test(NAME dsu_protocol_check
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/run_protocol_check.py
                $<TARGET_FILE:${EXECUTABLE_NAME}> --quick
    )
    # Binds the fixed DSU port 26760, so don't run alongside another server.
    set_tests_properties(dsu_protocol_check PROPERTIES RUN_SERIAL TRUE TIMEOUT 60)
else()
    message(STATUS "Tests: Python 3 not found, dsu_protocol_check not registered")
endif()
