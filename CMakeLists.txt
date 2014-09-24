cmake_minimum_required(VERSION 2.8)
project(wap_exam CXX)


###########################
# Settings related to g++ #
###########################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -std=c++11")


####################
# Setup googletest #
####################
add_subdirectory(lib/gtest-1.7.0)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}) 
enable_testing()

# function that create test cases.
function(add_gtest item)
  target_link_libraries(${item} gtest_main)
  add_test(${item} ${item})
endfunction(add_gtest)


#################
# Build runtime #
#################
include_directories(src)
include_directories(test)

option(deploy "" OFF)
if(deploy)
  add_executable(a.out src/orienteering.cpp)
else(deploy)
  add_subdirectory(test)
endif(deploy)
