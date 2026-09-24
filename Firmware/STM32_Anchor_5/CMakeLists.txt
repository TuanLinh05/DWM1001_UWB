cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32f103_dw1000_anchor_5)

# Reuse the same DW1000 register driver and protocol-v2 responder as A1-A4.
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/uwb_node.cmake)
uwb_node_setup(ANCHOR)
