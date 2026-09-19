cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dwm1001_anchor_8)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
  src/main.c
  src/platform/uwb_platform_zephyr.c
  src/drivers/dw1000.c
  src/ranging/anchor_ranging.c
)

target_compile_options(app PRIVATE
  -Wall
  -Wextra
  -Wformat=2
  -Wshadow
)
