cmake_minimum_required(VERSION 3.15)
project(standAloneKinematicsDemo1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED COMPONENTS thread system filesystem)
if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
endif()

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

coppeliasim_add_executable(standAloneKinematicsDemo1
    SOURCES
    standAloneKinematicsDemo1.cpp
    ../remoteApi/extApiPlatform.c
    ../remoteApi/extApi.c
    ../coppeliaKinematicsRoutines/ik.cpp
    ../coppeliaKinematicsRoutines/environment.cpp
    ../coppeliaKinematicsRoutines/serialization.cpp
    ../coppeliaKinematicsRoutines/ikRoutines.cpp
    ../coppeliaKinematicsRoutines/ikGroupContainer.cpp
    ../coppeliaKinematicsRoutines/ikGroup.cpp
    ../coppeliaKinematicsRoutines/ikElement.cpp
    ../coppeliaKinematicsRoutines/objectContainer.cpp
    ../coppeliaKinematicsRoutines/sceneObject.cpp
    ../coppeliaKinematicsRoutines/dummy.cpp
    ../coppeliaKinematicsRoutines/joint.cpp
    ../simMath/MyMath.cpp
    ../simMath/3Vector.cpp
    ../simMath/4Vector.cpp
    ../simMath/7Vector.cpp
    ../simMath/Vector.cpp
    ../simMath/3X3Matrix.cpp
    ../simMath/4X4Matrix.cpp
    ../simMath/4X4FullMatrix.cpp
    ../simMath/MMatrix.cpp
    ../common/shared_memory.c
)
target_compile_definitions(standAloneKinematicsDemo1 PRIVATE NON_MATLAB_PARSING)
target_compile_definitions(standAloneKinematicsDemo1 PRIVATE MAX_EXT_API_CONNECTIONS=255)
target_include_directories(standAloneKinematicsDemo1 PRIVATE ../remoteApi)
target_include_directories(standAloneKinematicsDemo1 PRIVATE ../simMath)
target_include_directories(standAloneKinematicsDemo1 PRIVATE ../include)
target_include_directories(standAloneKinematicsDemo1 PRIVATE ../coppeliaKinematicsRoutines)
target_link_libraries(standAloneKinematicsDemo1 Boost::system)
if(UNIX AND NOT APPLE)
    target_link_libraries(standAloneKinematicsDemo1 Threads::Threads rt)
endif()
