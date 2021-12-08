cmake_minimum_required(VERSION 3.9)
project(hinj VERSION 1.0.0 DESCRIPTION "provides abstractions for injecting faults")
include(GNUInstallDirs)

add_library(hinj SHARED libhinj.c)
set_target_properties(hinj PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      SOVERSION 0
                      PUBLIC_HEADER libhinj.h)

#add_executable(test test.c)
target_link_libraries(test hinj)

install(TARGETS hinj
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
