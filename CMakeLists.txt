cmake_minimum_required(VERSION 3.23)

include(cmake/prelude.cmake)

project(
    opengl-tutorial
    VERSION 0.1.0
    DESCRIPTION "Short description"
    HOMEPAGE_URL "https://github.com/dutiona/opengl-tutorial/"
    LANGUAGES CXX
)

docs_early_return()

include(cmake/project-is-top-level.cmake)
include(cmake/variables.cmake)

# ---- Declare library ----

add_library(
    opengl-tutorial_lib OBJECT
    source/game.cpp
)

target_include_directories(
    opengl-tutorial_lib ${opengl-tutorial_warning_guard}
    PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/source>"
)

target_compile_features(opengl-tutorial_lib PUBLIC cxx_std_20)


add_library(
    opengl-tutorial_glad OBJECT
    source/glad/src/glad.cpp
)

target_include_directories(
    opengl-tutorial_glad ${opengl-tutorial_warning_guard}
    PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/source/glad/include>"
)

target_link_libraries(opengl-tutorial_glad PRIVATE dl)

# ---- Declare executable ----

add_executable(opengl-tutorial_opengl-tutorial source/main.cpp)
add_executable(opengl-tutorial::opengl-tutorial ALIAS opengl-tutorial_opengl-tutorial)

set_target_properties(
    opengl-tutorial_opengl-tutorial PROPERTIES
    OUTPUT_NAME opengl-tutorial
    EXPORT_NAME opengl-tutorial
)

target_compile_features(opengl-tutorial_opengl-tutorial PRIVATE cxx_std_20)

find_package(ctre REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(opengl_system REQUIRED)
find_package(tl-expected REQUIRED)
find_package(range-v3 REQUIRED)
find_package(xsimd REQUIRED)

target_link_libraries(opengl-tutorial_opengl-tutorial
                      PRIVATE
                      opengl-tutorial_lib
                      opengl-tutorial_glad
                      ctre::ctre
                      Eigen3::Eigen
                      fmt::fmt
                      glfw
                      glm::glm
                      opengl::opengl
                      tl::expected
                      range-v3::range-v3
                      xsimd
)

# ---- Install rules ----

if(NOT CMAKE_SKIP_INSTALL_RULES)
  include(cmake/install-rules.cmake)
endif()

# ---- Developer mode ----

if(NOT opengl-tutorial_DEVELOPER_MODE)
  return()
elseif(NOT PROJECT_IS_TOP_LEVEL)
  message(
      AUTHOR_WARNING
      "Developer mode is intended for developers of opengl-tutorial"
  )
endif()

include(cmake/dev-mode.cmake)
