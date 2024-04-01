project(lns)

cmake_minimum_required (VERSION 3.16)
# cmake_minimum_required (VERSION 3.10)
set (CMAKE_CXX_STANDARD 17)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

# Find Eigen3 for PIBT
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(BOOST_THREAD_LIBRARY boost_thread -pthread)

if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  # set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O3")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release") 

# target_link_libraries(lns ${Boost_LIBRARIES} Eigen3::Eigen)

# find_package(TBB REQUIRED)

# add_subdirectory(./lacam3)

## parallel lns
include_directories("src" "src/CBS" "src/PIBT" "src/utilities" "src/sharing" "src/lacam3/include")
file(GLOB SOURCES "src/*.cpp" "src/CBS/*.cpp" "src/PIBT/*.cpp" "src/lacam3/src/*.cpp"
     "src/*.cpp" "src/utilities/*.cpp" "src/sharing/*.cpp")
add_executable(plns ${SOURCES})
target_compile_features(plns PUBLIC cxx_std_17)
target_link_libraries(plns ${Boost_LIBRARIES} 
    Eigen3::Eigen
    libpthread.so
    # profiler
    # tcmalloc
    # TBB::tbb -ltbbmalloc
)