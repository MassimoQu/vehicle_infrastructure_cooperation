cmake_minimum_required(VERSION 2.6)
project(TEST)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local")
find_package(PCL 1.12 REQUIRED)
# include_directories("usr/include/X11/Xlib.h")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBS})
# message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV_LIB_DIRS: ${OpenCV_LIBS}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_LIBRARIES})
message(STATUS "jsoncpp_INS: ${JSONCPP_LIBRARIES}")


add_executable(TEST test_true_boxing.cpp)

target_link_libraries (TEST ${PCL_LIBRARIES})
target_link_libraries( TEST  ${OpenCV_LIBS})
target_link_libraries(TEST ${JSONCPP_LIBRARIES})
message(STATUS "jsoncpp_LIBRARIES: ${JSONCPP_LIBRARIES}")
 