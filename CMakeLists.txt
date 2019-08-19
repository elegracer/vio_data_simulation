cmake_minimum_required(VERSION 3.13)

project(vio_data_simulation
    VERSION 0.0.1
    LANGUAGES CXX
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(viosim)
target_sources(viosim
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/viosim/imu.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/viosim/param.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/viosim/utility.cpp
)
target_include_directories(viosim
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)
target_compile_features(viosim
    PUBLIC
        cxx_std_17
)
target_link_libraries(viosim
    PUBLIC
        Eigen3::Eigen
)


add_executable(gen_data)
target_sources(gen_data
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/utils/generate_data.cpp
)
target_include_directories(gen_data
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gen_data
    PRIVATE
        viosim
        ${OpenCV_LIBS}
)


add_executable(gen_static_data)
target_sources(gen_static_data
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/utils/generate_static_data.cpp
)
target_include_directories(gen_static_data
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(gen_static_data
    PRIVATE
        viosim
        ${OpenCV_LIBS}
)
