from conans import ConanFile, CMake

class MathConan(ConanFile):
    name = "lms_math"
    version = "1.0"
    settings = "os", "compiler", "build_type", "arch"
    exports = "include/*","src/*","README.md","CMakeLists.txt"
    requires = "gtest/1.8.0@lms/stable","lms/2.0@lms/stable","cereal/1.2-0@lms/stable","Eigen3/3.2.8@bilke/stable"
    generators = "cmake"

    def build(self):
        cmake = CMake(self.settings)
        self.run('cmake %s %s' % (self.conanfile_directory, cmake.command_line))
        self.run("cmake --build . %s" % cmake.build_config)

    def package(self):
        self.copy("*.h", dst="include", src="include")
        self.copy("*.so", dst="lib")

    def package_info(self):
        self.cpp_info.libs = ["lms_math"]
