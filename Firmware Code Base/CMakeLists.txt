cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dwm1001_uwb)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
  src/main.c
  src/platform/uwb_platform_zephyr.c
  src/drivers/dw1000.c
)

if(CONFIG_UWB_ROLE_TAG)
  target_sources(app PRIVATE
    src/ranging/tag_ranging.c
    src/filters/range_filter.c
    src/telemetry/telemetry.c
    src/telemetry/uart_tx_zephyr.c
  )
elseif(CONFIG_UWB_ROLE_ANCHOR)
  target_sources(app PRIVATE src/ranging/anchor_ranging.c)
else()
  target_sources(app PRIVATE
    src/diagnostics/hardware_test.c
    src/telemetry/uart_tx_zephyr.c
  )
endif()

target_compile_options(app PRIVATE
  -Wall
  -Wextra
  -Wformat=2
  -Wshadow
)
