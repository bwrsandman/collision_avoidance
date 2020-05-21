file(GLOB FILES
  *.cpp
  *.h
  imgui_sdl/imgui_sdl.cpp
  imgui_sdl/imgui_sdl.h
  examples/imgui_impl_sdl.cpp
  examples/imgui_impl_sdl.h)

add_library(imgui STATIC ${FILES})
target_include_directories(imgui
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui_sdl
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui_club
    ${CMAKE_CURRENT_SOURCE_DIR}/examples)

if (EMSCRIPTEN)
  set_target_properties(imgui PROPERTIES
    COMPILE_FLAGS_DEBUG "-g4"
    COMPILE_FLAGS "-s USE_SDL=2 --emrun"
    LINK_FLAGS_DEBUG "-g4"
    LINK_FLAGS "-s USE_SDL=2 --emrun"
  )
else()
  find_package(SDL2 REQUIRED)
  target_link_libraries(imgui PUBLIC SDL2::SDL2)
endif()
