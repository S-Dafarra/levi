cmake_minimum_required(VERSION 3.5)

# ouptut paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Build shared libs
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Enable C++14
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include macros for warnings
include(AddWarningsConfigurationToTargets)

include(CMakePackageConfigHelpers)

project(sDiff
        LANGUAGES CXX
        VERSION 0.0.100)

include(GNUInstallDirs)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
      message(STATUS "Setting build type to 'Release' as none was specified.")
      set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#set(SDIFF_SOURCES src/Evaluable.cpp
#                  src/Expression.cpp
#                  src/OperatorsEvaluables.cpp)

#set(SDIFF_HEADERS include/sDiff/Evaluable.h
#                  include/sDiff/Expression.h
#                  include/sDiff/Variable.h
#                  include/sDiff/OperatorsEvaluables.h
#                  include/sDiff/sDiff.h)

#add_library(sDiff ${SDIFF_HEADERS} ${SDIFF_SOURCES})
add_library(sDiff INTERFACE)
target_include_directories(sDiff INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(sDiff INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

#set_property(TARGET sDiff PROPERTY PUBLIC_HEADER ${SDIFF_HEADERS})

# install
install(TARGETS sDiff
        EXPORT sDiff
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT sDiff
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sDiff)

configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/sDiffConfig.cmake.in
                              ${CMAKE_BINARY_DIR}/sDiffConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sDiff)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/sDiffConfigVersion.cmake  VERSION ${sDiff_VERSION} COMPATIBILITY SameMajorVersion )

install(FILES ${CMAKE_BINARY_DIR}/sDiffConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/sDiffConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sDiff)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include)

include(AddUninstallTarget)

include(CTest)
if(BUILD_TESTING)
   add_subdirectory(tests)
endif()


