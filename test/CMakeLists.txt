include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(utils_test utils_test.cpp)
target_link_libraries(utils_test gtest_main)

add_executable(unicycle_test unicycle_test.cpp)
target_link_libraries(unicycle_test gtest_main unicycle)

include(GoogleTest)
gtest_discover_tests(utils_test)
gtest_discover_tests(unicycle_test)
