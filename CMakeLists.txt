cmake_minimum_required(VERSION 3.13)
project(space_junk_simulator)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(space_junk_simulator
        src/cpu/cpu_solver.cpp
        src/cpu/cpu_solver.h
        src/cpu_solver_main.cpp
        src/gpu/gpu_solver.h
        lib/json.hpp
        src/model.cpp
        src/model.h
        test/test.cpp)
