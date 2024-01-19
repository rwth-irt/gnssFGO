project(gnss_tools)
cmake_minimum_required(VERSION 3.02)

message(STATUS "test: subdirs(${CMAKE_CURRENT_SOURCE_DIR})")

MACRO(list_sub_dir result firstdir curdir)
    file(GLOB ENDF6_SRC_TOP RELATIVE
            ${curdir} ${curdir}/*)
    file(GLOB_RECURSE ENDF6_SRC_NESTED ${curdir}/*)
    set(children ${ENDF6_SRC_TOP} ${ENDF6_SRC_NESTED})

    SET(dirlist "${firstdir}")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child} AND NOT ${child} STREQUAL "cmake-build-debug")
            LIST(APPEND dirlist ${curdir}/${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

list_sub_dir(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-fPIC)
set(tools_lib_list "")

foreach(SUBDIR ${SUBDIRS})
    get_filename_component(MODULE_NAME ${SUBDIR} NAME)
    IF(NOT ${MODULE_NAME} STREQUAL "")
        message(STATUS "Building Module: ${MODULE_NAME}")
        file(GLOB_RECURSE MODULE_SRCS "${SUBDIR}/*.cpp")
        file(GLOB_RECURSE MODULE_HRDS"${SUBDIR}/*.h")
        add_library(_${MODULE_NAME} STATIC ${MODULE_SRCS} matlab_utils.h)
        target_link_libraries(_${MODULE_NAME} ${MODULE_HRDS})
        LIST(APPEND tools_lib_list _${MODULE_NAME})
    ENDIF()
endforeach(SUBDIR)
set(tools_lib_list ${tools_lib_list} PARENT_SCOPE)


