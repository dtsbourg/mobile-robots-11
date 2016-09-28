# Install script for directory: /Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/symbolicR

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug/libProject_symbolic.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug" TYPE STATIC_LIBRARY FILES "/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/symbolicR/libProject_symbolic.a")
  if(EXISTS "$ENV{DESTDIR}/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug/libProject_symbolic.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug/libProject_symbolic.a")
    execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}/Users/Dylan/Documents/Cours/MT_MA1/Mobile robots/Differential wheeled robot project files-20160928/m454_project/workR/build/Debug/libProject_symbolic.a")
  endif()
endif()

