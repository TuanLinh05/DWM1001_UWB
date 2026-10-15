# Shared build description for every DWM1001 / STM32 node project.
#
# Each node directory (nodes/tag/*, nodes/anchors/*/*, nodes/sniffer/*) only
# keeps its board overlay, prj.conf and include/uwb_app_config.h. All sources
# live in Firmware/common so a fix can never be applied to one copy and missed
# in another.
#
# Firmware/common is split by module; every module directory holds its .c
# files next to its .h files:
#   app/        main_tag.c, main_anchor.c, main_sniffer.c (entry points)
#   drivers/    DW1000 register driver
#   platform/   Zephyr glue (SPI, GPIO, IRQ, time)
#   ranging/    TAG / anchor state machines and air frames
#   filters/    range conditioner
#   telemetry/  binary UART uplink
#   services/   command channel, NVS settings, health
#   config/     calibration tables shared by every role
#
# Usage from a node CMakeLists.txt, after project():
#   include(${CMAKE_CURRENT_SOURCE_DIR}/<..>/common/cmake/uwb_node.cmake)
#   uwb_node_setup(TAG)      # or ANCHOR / SNIFFER

get_filename_component(UWB_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(UWB_FIRMWARE_DIR "${UWB_COMMON_DIR}/.." ABSOLUTE)
set(UWB_COMMON_MODULES app drivers platform ranging filters telemetry services config)

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
  set(material "role=${role}\nboard=${BOARD}\nhigh_rate=${UWB_HIGH_RATE}\n")
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
  if(UWB_HIGH_RATE)
    # Include sources as well as configuration: dirty builds from the same
    # commit must not advertise the previous package's config identity.
    foreach(relative IN ITEMS
        cmake/uwb_node.cmake drivers/dw1000_hw.h ranging/anchor_ranging.h
        ranging/tag_ranging.h platform/uwb_platform_zephyr.c
        drivers/dw1000.c ranging/anchor_ranging.c
        ranging/tag_ranging.c app/main_tag.c app/main_anchor.c
        ranging/uwb_frame.h platform/uwb_platform.h services/uwb_health.h
        ranging/uwb_frame.c filters/range_filter.c
        services/uwb_cmd.c services/uwb_health.c services/uwb_settings.c
        telemetry/telemetry.c telemetry/telemetry_frame.c
        telemetry/uart_tx_zephyr.c)
      file(SHA256 "${UWB_COMMON_DIR}/${relative}" source_hash)
      string(APPEND material "${relative}=${source_hash}\n")
    endforeach()
    foreach(profile IN ITEMS highrate_common.conf highrate_stm32.overlay)
      file(SHA256 "${UWB_FIRMWARE_DIR}/profiles/${profile}" extra_hash)
      string(APPEND material "${profile}=${extra_hash}\n")
    endforeach()
    file(SHA256 "${UWB_FIRMWARE_DIR}/profiles/highrate_nrf.overlay" profile_hash)
    string(APPEND material "highrate_overlay=${profile_hash}\n")
    if(CONFIG_SOC_NRF52832)
      file(SHA256 "${UWB_FIRMWARE_DIR}/profiles/highrate_nrf.conf" profile_conf_hash)
      string(APPEND material "highrate_conf=${profile_conf_hash}\n")
    endif()
  endif()
  if(UWB_HIGH_RATE_300)
    string(APPEND material "high_rate_300=${UWB_HIGH_RATE_300}\n")
  endif()
  string(SHA256 config_hash "${material}")
  string(SUBSTRING "${config_hash}" 0 8 config_hash_short)
  set(${out_hash} "0x${config_hash_short}" PARENT_SCOPE)
endfunction()

function(uwb_node_setup role)
  if(UWB_HIGH_RATE_300)
    set(UWB_HIGH_RATE ON)
    target_compile_definitions(app PRIVATE UWB_HIGH_RATE_300=1 UWB_PHY_PREAMBLE=64U)
  endif()
  if(UWB_HIGH_RATE)
    target_compile_definitions(app PRIVATE UWB_HIGH_RATE=1 UWB_BURST_REQUIRE_DS=1)
    if(CONFIG_SOC_NRF52832)
      target_compile_definitions(app PRIVATE UWB_DIRECT_SPIM=1)
    elseif(CONFIG_SOC_SERIES_STM32F1X)
      target_compile_definitions(app PRIVATE UWB_DIRECT_STM32_SPI=1 DW_SPI_FAST_MHZ=16U)
    endif()
  endif()
  # The node's own include/ directory comes first: it provides uwb_app_config.h.
  # Then every common module directory, so sources keep flat #include "x.h".
  set(common_include_dirs)
  foreach(module IN LISTS UWB_COMMON_MODULES)
    list(APPEND common_include_dirs ${UWB_COMMON_DIR}/${module})
  endforeach()
  target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${common_include_dirs})

  set(radio_sources
    ${UWB_COMMON_DIR}/platform/uwb_platform_zephyr.c
    ${UWB_COMMON_DIR}/drivers/dw1000.c
    ${UWB_COMMON_DIR}/ranging/uwb_frame.c
    ${UWB_COMMON_DIR}/services/uwb_health.c)

  if(role STREQUAL "TAG")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/app/main_tag.c
      ${UWB_COMMON_DIR}/services/uwb_cmd.c
      ${UWB_COMMON_DIR}/services/uwb_settings.c
      ${UWB_COMMON_DIR}/ranging/tag_ranging.c
      ${UWB_COMMON_DIR}/filters/range_filter.c
      ${UWB_COMMON_DIR}/telemetry/telemetry.c
      ${UWB_COMMON_DIR}/telemetry/telemetry_frame.c
      ${UWB_COMMON_DIR}/telemetry/uart_tx_zephyr.c)
    target_compile_definitions(app PRIVATE UWB_ROLE_TAG=1)
  elseif(role STREQUAL "ANCHOR")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/app/main_anchor.c
      ${UWB_COMMON_DIR}/ranging/anchor_ranging.c)
    target_compile_definitions(app PRIVATE UWB_ROLE_ANCHOR=1)
  elseif(role STREQUAL "SNIFFER")
    target_sources(app PRIVATE
      ${radio_sources}
      ${UWB_COMMON_DIR}/app/main_sniffer.c
      ${UWB_COMMON_DIR}/telemetry/telemetry_frame.c
      ${UWB_COMMON_DIR}/telemetry/uart_tx_zephyr.c)
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
