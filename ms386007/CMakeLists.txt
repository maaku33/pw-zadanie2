cmake_minimum_required(VERSION 2.8)
project(pw-zadanie2 C CXX)

find_package ( Threads REQUIRED )

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pthread -Wall -Wextra -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -pthread -O3")

add_executable(adorate adorate.cpp blimit.cpp)

install(TARGETS DESTINATION .)
