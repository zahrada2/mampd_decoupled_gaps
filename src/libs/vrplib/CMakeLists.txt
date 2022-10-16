set(VRPLIB_HEADERS
        # logging.h
        vrpnode.hpp
        vrplib_loader.hpp
        vrplib.hpp
        )

add_library(vrplib
        ${VRPLIB_HEADERS}
        vrpnode.cpp
        vrplib_loader.cpp
        vrplib.cpp
        )

target_link_libraries(vrplib ${Boost_LIBRARIES})
