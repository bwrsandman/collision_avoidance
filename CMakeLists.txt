cmake_minimum_required(VERSION 3.15)
project(collision_avoidance)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/RVO2-2.0.2)
add_subdirectory(third_party/imgui-1.74)

add_executable(collision_avoidance main.cpp)
target_link_libraries(collision_avoidance PRIVATE RVO imgui)

if (EMSCRIPTEN)
  set_target_properties(collision_avoidance PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1 --emrun"
    SUFFIX ".html"
  )
else()
  find_package(SDL2 REQUIRED)
  target_link_libraries(collision_avoidance PRIVATE SDL2::SDL2)
endif()
