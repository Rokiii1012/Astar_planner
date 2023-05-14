# Install script for directory: /home/fyk/eigen/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/fyk/eigen/Eigen/Cholesky"
    "/home/fyk/eigen/Eigen/CholmodSupport"
    "/home/fyk/eigen/Eigen/Core"
    "/home/fyk/eigen/Eigen/Dense"
    "/home/fyk/eigen/Eigen/Eigen"
    "/home/fyk/eigen/Eigen/Eigenvalues"
    "/home/fyk/eigen/Eigen/Geometry"
    "/home/fyk/eigen/Eigen/Householder"
    "/home/fyk/eigen/Eigen/IterativeLinearSolvers"
    "/home/fyk/eigen/Eigen/Jacobi"
    "/home/fyk/eigen/Eigen/LU"
    "/home/fyk/eigen/Eigen/MetisSupport"
    "/home/fyk/eigen/Eigen/OrderingMethods"
    "/home/fyk/eigen/Eigen/PaStiXSupport"
    "/home/fyk/eigen/Eigen/PardisoSupport"
    "/home/fyk/eigen/Eigen/QR"
    "/home/fyk/eigen/Eigen/QtAlignedMalloc"
    "/home/fyk/eigen/Eigen/SPQRSupport"
    "/home/fyk/eigen/Eigen/SVD"
    "/home/fyk/eigen/Eigen/Sparse"
    "/home/fyk/eigen/Eigen/SparseCholesky"
    "/home/fyk/eigen/Eigen/SparseCore"
    "/home/fyk/eigen/Eigen/SparseLU"
    "/home/fyk/eigen/Eigen/SparseQR"
    "/home/fyk/eigen/Eigen/StdDeque"
    "/home/fyk/eigen/Eigen/StdList"
    "/home/fyk/eigen/Eigen/StdVector"
    "/home/fyk/eigen/Eigen/SuperLUSupport"
    "/home/fyk/eigen/Eigen/UmfPackSupport"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/fyk/eigen/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

