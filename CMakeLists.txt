cmake_minimum_required(VERSION 3.27)
project(Graphics_TinyRenderer)

set(CMAKE_CXX_STANDARD 17)

add_executable(Graphics_TinyRenderer main.cpp
        model.cpp
        model.h
        tgahandler.cpp
        tgahandler.h
        geometry.h
        geometry.cpp
        renderer.cpp
        renderer.h)
