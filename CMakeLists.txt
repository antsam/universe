cmake_minimum_required(VERSION 2.6)
project(universe)

set(universe_HEADERS src/universe.h)
set(universe_SOURCES src/universe.cc src/controller.cc)

find_package(GLUT REQUIRED)

if (NOT APPLE)
    find_package(OpenGL REQUIRED)
    if (NOT ${OPENGL_GLU_FOUND} STREQUAL "YES")
        message( FATAL_ERROR "GLU not found" )
    endif (NOT ${OPENGL_GLU_FOUND} STREQUAL "YES")
else (NOT APPLE)
    set(OPENGL_INCLUDE_DIR /usr/X11R6/include/)
    set(OPENGL_LIBRARIES /usr/X11R6/lib/libGL.dylib /usr/X11R6/lib/libGLU.dylib)
endif (NOT APPLE)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)
add_executable(universe 
    ${universe_HEADERS}
    ${universe_SOURCES}
)

include_directories(${GLUT_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS} ${OpenGL_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS} ${OpenGL_DEFINITIONS})

target_link_libraries(universe ${GLUT_LIBRARY} ${OPENGL_LIBRARIES})