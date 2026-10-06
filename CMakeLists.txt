cmake_minimum_required(VERSION 3.7)
project(Filters)

include(filters.cmake)

add_definitions(-fpic)

SET(LIBDAV2D_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/dec_av2.c
)

SET(LIBDAV2D_LIB
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdav2d.a
)

SET(LIBDAV2D_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_filter(libdav2d
        "${LIBDAV2D_SRC}"
        "${LIBDAV2D_LIB}"
        []
        ""
        "${LIBDAV2D_INC}"
        ""
        "1")
