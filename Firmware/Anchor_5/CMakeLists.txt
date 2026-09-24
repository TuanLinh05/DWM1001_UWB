cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dwm1001_anchor_5)

# All sources are shared through Firmware/common; this directory only holds
# the node configuration (prj.conf, app.overlay, include/uwb_app_config.h).
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/uwb_node.cmake)
uwb_node_setup(ANCHOR)
