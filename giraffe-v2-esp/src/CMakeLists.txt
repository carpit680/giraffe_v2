# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
set(component_srcs "rotary_encoder_pcnt_ec11.c")

# idf_component_register(SRCS ${app_sources})
idf_component_register(SRCS ${app_sources} ${component_srcs}
                    #    INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
