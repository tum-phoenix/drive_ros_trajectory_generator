if(USE_CONAN)
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
endif(USE_CONAN)

set ( SOURCES
    "src/stub.cpp"
    "src/curve.cpp"
    "src/math.cpp"
)

set (HEADERS
    "include/lms/math/interpolation.h"
    "include/lms/math/lookup_table.h"
    "include/lms/math/vertex.h"
    "include/lms/math/polyline.h"
    "include/lms/math/curve.h"
    "include/lms/math/math.h"
    "include/lms/math/convert.h"
    "include/lms/math/mathEigen.h"
    "include/lms/math/point_cloud.h"
    "include/lms/math/pose.h"
)

set (TESTS
    "src/test/lookup_table.cpp"
    "src/test/interpolation.cpp"
    "src/test/convert.cpp"
    "src/test/vertex.cpp"
    "src/test/pose2d.cpp"
    "src/test/polyline2f.cpp"
)

#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

include_directories("include")

add_library(lms_math SHARED ${SOURCES} ${HEADERS})

if(USE_CONAN)
target_link_libraries(lms_math PRIVATE ${CONAN_LIBS})
message("Compile math_lib testing executable")
add_executable(math_lib_test ${TESTS})
target_link_libraries(math_lib_test PRIVATE lms_math ${CONAN_LIBS})
add_test(MathLib math_lib_test)
else()
target_link_libraries(lms_math PRIVATE lmscore)

#non conan test
#message("Compile math_lib testing executable")
#add_executable(math_lib_test ${TESTS})
#target_link_libraries(math_lib_test PRIVATE lms_math gtest gmock gmock_main)
#add_test(MathLib math_lib_test)

endif(USE_CONAN)
