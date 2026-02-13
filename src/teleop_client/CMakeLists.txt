find_program(CARGO_CMD cargo)
if (NOT CARGO_CMD)
    message(FATAL_ERROR "Rust cargo not found! Please install Rust.")
endif ()

set(RUST_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_custom_target(teleop_client_build ALL
        COMMAND ${CARGO_CMD} build --release
        WORKING_DIRECTORY ${RUST_PROJECT_DIR}
        COMMENT "Building Rust GUI (teleop_client)..."
)

add_custom_command(TARGET teleop_client_build POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RUST_PROJECT_DIR}/target/release/teleop_client${CMAKE_EXECUTABLE_SUFFIX}"
        "${CMAKE_CURRENT_BINARY_DIR}/teleop_client${CMAKE_EXECUTABLE_SUFFIX}"
)