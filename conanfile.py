from conans import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, CMake


class OpenGlTuto(ConanFile):
    name = "opengl-tutorial"
    version = "head"
    license = "MIT"
    url = "https://github.com/dutiona/opengl-tutorial"
    settings = "os", "compiler", "arch", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": False,
        "gtest:shared": False,
    }

    generators = ["CMakeDeps", "CMakeToolchain"]

    build_requires = [
        "gtest/[>=1.11.0]",
        "benchmark/[>=1.5.0]",
    ]

    requires = [
        "range-v3/0.11.0",
        "glfw/3.3.7",
        "fmt/8.1.1",
        "ctre/3.7",
        "tl-expected/20190710",
        "xsimd/8.1.0",
        "eigen/3.4.0",
        "glm/0.9.9.8",
        # "glad/0.1.35",
        "opengl/system",
        # "boost/1.79.0",
    ]

    def generate(self):
        cmake = CMakeDeps(self)
        cmake.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.install()

    def package_id(self):
        del self.info.settings.compiler.cppstd
