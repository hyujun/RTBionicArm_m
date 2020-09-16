cmake_minimum_required(VERSION 2.8)
project(KDL)

add_library(${PROJECT_NAME} SHARED STATIC
        LieDynamics.cpp
        LieOperator.cpp
        PoEKinematics.cpp
        PropertyDefinition.cpp
        SerialManipulator.cpp
        )

add_library(sub::KDL ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

#find_package(Eigen3 CONFIG)
set( EIGEN3_INCLUDE_DIR /usr/include/eigen3)
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
include_directories ( ${EIGEN3_INCLUDE_DIR} )
message(STATUS "Eigen3 includes : ${EIGEN3_INCLUDE_DIR}" )