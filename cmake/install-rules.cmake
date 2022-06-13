if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_INSTALL_INCLUDEDIR include/opengl-tutorial CACHE PATH "")
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# find_package(<package>) call for consumers to find this project
set(package opengl-tutorial)

install(
    TARGETS opengl-tutorial_opengl-tutorial
    EXPORT opengl-tutorialTargets
    RUNTIME COMPONENT opengl-tutorial_Runtime
)

write_basic_package_version_file(
    "${package}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

# Allow package maintainers to freely override the path for the configs
set(
    opengl-tutorial_INSTALL_CMAKEDIR "${CMAKE_INSTALL_DATADIR}/${package}"
    CACHE PATH "CMake package config location relative to the install prefix"
)
mark_as_advanced(opengl-tutorial_INSTALL_CMAKEDIR)

install(
    FILES cmake/install-config.cmake
    DESTINATION "${opengl-tutorial_INSTALL_CMAKEDIR}"
    RENAME "${package}Config.cmake"
    COMPONENT opengl-tutorial_Development
)

install(
    FILES "${PROJECT_BINARY_DIR}/${package}ConfigVersion.cmake"
    DESTINATION "${opengl-tutorial_INSTALL_CMAKEDIR}"
    COMPONENT opengl-tutorial_Development
)

install(
    EXPORT opengl-tutorialTargets
    NAMESPACE opengl-tutorial::
    DESTINATION "${opengl-tutorial_INSTALL_CMAKEDIR}"
    COMPONENT opengl-tutorial_Development
)

if(PROJECT_IS_TOP_LEVEL)
  include(CPack)
endif()
