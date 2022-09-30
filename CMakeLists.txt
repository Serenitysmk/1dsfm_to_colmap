cmake_minimum_required(VERSION 3.0)

project(1DSfMToCOLMAP)

find_package(COLMAP REQUIRED)

add_executable(read_1dsfm read_1dsfm.cc)
target_include_directories(read_1dsfm PRIVATE ${COLMAP_INCLUDE_DIRS})
target_link_directories(read_1dsfm PRIVATE ${COLMAP_LINK_DIRS})
target_link_libraries(read_1dsfm PRIVATE ${COLMAP_LIBRARIES})