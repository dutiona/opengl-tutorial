# Parent project does not export its library target, so this CML implicitly
# depends on being added from it, i.e. the testing is done only from the build
# tree and is not feasible from an install location

project(opengl-tutorialTests LANGUAGES CXX)

add_executable(opengl-tutorial_test source/opengl-tutorial_test.cpp)
target_link_libraries(opengl-tutorial_test PRIVATE opengl-tutorial_lib)
target_compile_features(opengl-tutorial_test PRIVATE cxx_std_23)

add_test(NAME opengl-tutorial_test COMMAND opengl-tutorial_test)
