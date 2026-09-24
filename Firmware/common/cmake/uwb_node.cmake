# Shared build description for every DWM1001 node project.
#
# Each node directory (Tag, Tag_DevKit, Anchor_N, Sniffer_DevKit) only keeps
# its board overlay, prj.conf and include/uwb_app_config.h. All sources live in
# Firmware/common so a fix can never be applied to one copy and missed in
# another.
#
# Usage from a node CMakeLists.txt, after project():
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/uwb_node.cmake)
#   uwb_node_setup(TAG)      # or ANCHOR / SNIFFER

get_filename_component(UWB_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# Build identity. The Windows build scripts map Firmware/ to a drive letter
# (Kconfig cannot handle spaces), which hides the repository's .git directory
# from the mapped path, so they pass the real repository root explicitly.
function(uwb_read_build_id out_hash out_dirty)
  set(hash 0)
  set(dirty 1)
  if(DEFINED ENV{UWB_REPO_ROOT})
    set(repo "$ENV{UWB_REPO_ROOT}")
  else()
    set(repo "${UWB_COMMON_DIR}")
  endif()
  find_package(Git QUIET)
  if(GIT_FOUND)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${repo}" rev-parse --short=8 HEAD
      OUTPUT_VARIABLE git_hash
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE git_result
      ERROR_QUIET)
    if(git_result EQUAL 0 AND git_hash MATCHES "^[0-9a-fA-F]+$")
      set(hash "0x${git_hash}")
      execute_process(
        # Untracked sources are build inputs too. Excluding them could produce
        # a binary from files that are absent from the advertised commit while
        # still reporting dirty=0.
        COMMAND "${GIT_EXECUTABLE}" -C "${repo}" status --porcelain --untracked-files=normal
        OUTPUT_VARIABLE git_status
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE status_result
        ERROR_QUIET)
      if(status_result EQUAL 0 AND git_status STREQUAL "")
        set(dirty 0)
      endif()
    endif()
  endif()
  set(${out_hash} ${hash} PARENT_SCOPE)
  set(${out_dirty} ${dirty} PARENT_SCOPE)
endfunction()

# Reproducible identity for node-local configuration that is not represented
# by the shared source commit alone. SHA-256 is truncated to 32 bits for the
# compact DEVICE_INFO payload; the full files remain the release authority.
function(uwb_read_config_id role out_hash)
  set(material "role=${role}\nboard=${BOARD}\n")
  foreach(relative IN ITEMS
      CMakeLists.txt
      prj.conf
      app.overlay
      include/uwb_app_config.h)
    set(path "${CMAKE_CURRENT_SOURCE_DIR}/${relative}")
    if(EXISTS "${path}")
      file(SHA256 "${path}" file_hash)
      string(APPEND material "${relative}=${file_hash}\n")
    else()
      string(APPEND material "${relative}=MISSING\n")
    endif()
  endforeach()
  string(SHA256 config_hash "${material}")
  string(SUBSTRING "${config_hash}" 0 8 config_hash_short)
  set(${out_hash} "0x${config_hash_short}" PARENT_SCOPE)
endfunction()

function(uwb_node_setup role)
  # The node's own include/ directory comes first: it provides uwb_app_config.h.
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UWB_COMMON_DIR}/include)

  set(radio_sources
    ${UWB_COMMON_DIR}/src/platform/uwb_platform_zephyr.c
    ${UWB_COMMON_DIR}/src/drivers/dw1000.c
    ${UWB_COMMON_DIR}/src/ranging/uwb_frame.c
    ${UWB_COMMON_DIR}/src/app/uwb_health.c)

  if(role STREQUAL "TAG")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/src/app/main_tag.c
      ${UWB_COMMON_DIR}/src/app/uwb_cmd.c
      ${UWB_COMMON_DIR}/src/app/uwb_settings.c
      ${UWB_COMMON_DIR}/src/ranging/tag_ranging.c
      ${UWB_COMMON_DIR}/src/filters/range_filter.c
      ${UWB_COMMON_DIR}/src/telemetry/telemetry.c
      ${UWB_COMMON_DIR}/src/telemetry/telemetry_frame.c
      ${UWB_COMMON_DIR}/src/telemetry/uart_tx_zephyr.c)
    target_compile_definitions(app PRIVATE UWB_ROLE_TAG=1)
  elseif(role STREQUAL "ANCHOR")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/src/app/main_anchor.c
      ${UWB_COMMON_DIR}/src/ranging/anchor_ranging.c)
    target_compile_definitions(app PRIVATE UWB_ROLE_ANCHOR=1)
  elseif(role STREQUAL "SNIFFER")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/src/app/main_sniffer.c
      ${UWB_COMMON_DIR}/src/telemetry/telemetry_frame.c
      ${UWB_COMMON_DIR}/src/telemetry/uart_tx_zephyr.c)
    target_compile_definitions(app PRIVATE UWB_ROLE_SNIFFER=1)
  else()
    message(FATAL_ERROR "uwb_node_setup: unknown role '${role}'")
  endif()

  uwb_read_build_id(build_hash build_dirty)
  uwb_read_config_id("${role}" build_config_hash)
  message(STATUS "UWB node role ${role}, build id ${build_hash} (dirty=${build_dirty}), config ${build_config_hash}")
  target_compile_definitions(app PRIVATE
    UWB_BUILD_GIT_HASH=${build_hash}
    UWB_BUILD_GIT_DIRTY=${build_dirty}
    UWB_BUILD_CONFIG_HASH=${build_config_hash})

  target_compile_options(app PRIVATE
    -Wall
    -Wextra
    -Wformat=2
    -Wshadow)
endfunction()
