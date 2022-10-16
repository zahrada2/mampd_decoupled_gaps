add_library(${PROJECT_NAME}_LIB
        utils.cpp
        structures.cpp
        Solution.cpp
        solutions_impl.cpp
        constructives.cpp
        local_search.cpp
        methods.cpp
        Solver.cpp
        MAPF_interface.cpp
	MapData.cpp
	MAPF/MAPF_SIPP.cpp
	MAPF/MAPF_PBS.cpp
	MAPF/SIPP.cpp
        )

target_link_libraries(${PROJECT_NAME}_LIB
        ${Boost_LIBRARIES})
