cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

include(FetchContent)

# Set DOWNLOAD_EXTRACT_TIMESTAMP by default
cmake_policy(SET CMP0135 NEW)

# libtorch 2.2.2-cpu
# ================================================================
# set(LIBTORCH_URL      "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.2.2%2Bcpu.zip")
# set(LIBTORCH_URL_HASH "SHA256=cb0a7238c5e6959ba352c0d6a75d4d4794a182ee34e6a0a0e275148035f8d855")

# libtorch 2.7.1-cpu
# ================================================================
set(LIBTORCH_URL      "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcpu.zip")
set(LIBTORCH_URL_HASH "SHA256=63d572598c8d532128a335018913e795c1bbb32602ce378896dc8cfbb5590976")

FetchContent_Declare(
  libtorch
  URL      ${LIBTORCH_URL}
  URL_HASH ${LIBTORCH_URL_HASH}
)

FetchContent_MakeAvailable(libtorch)

message(STATUS "Adding libtorch prefix to CMAKE_PREFIX_PATH: ${libtorch_SOURCE_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${libtorch_SOURCE_DIR}")

install(DIRECTORY ${libtorch_SOURCE_DIR}/include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h*"
)

install(DIRECTORY ${libtorch_SOURCE_DIR}/lib/
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${libtorch_SOURCE_DIR}/share/
    DESTINATION share/${PROJECT_NAME}
)
