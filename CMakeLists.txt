cmake_minimum_required(VERSION 3.27)
project(Graphics_TinyRenderer)

set(CMAKE_CXX_STANDARD 20)

add_executable(Graphics_TinyRenderer main.cpp
        model.cpp
        model.h
        tga-handler.cpp
        tga-handler.h
        geometry.h
        renderer.cpp
        renderer.h
        color.h
        shader.h
)
