cmake_minimum_required( VERSION 2.8 )

project( SuboptimalSIPP )

set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ann_1.1.2/include/ANN" )
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ann_1.1.2/src" )

link_directories( "${CMAKE_CURRENT_SOURCE_DIR}/ann_1.1.2/lib" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp  
     tinyxml2.cpp
     xmlLogger.cpp 
     mission.cpp 
     map.cpp 
     task.cpp
     config.cpp 
     dynamicobstacles.cpp
     aa_sipp.cpp
     constraints.cpp )

set( HEADER_FILES 
     tinyxml2.h
     heuristic.h
     searchresult.h 
     gl_const.h 
     xmlLogger.h 
     mission.h 
     map.h 
     task.h
     dynamicobstacles.h
     config.h
     lineofsight.h
     aa_sipp.h 
     structs.h
     constraints.h )

add_executable( SuboptimalSIPP ${SOURCE_FILES} ${HEADER_FILES} )

target_link_libraries( SuboptimalSIPP "${CMAKE_CURRENT_SOURCE_DIR}/ann_1.1.2/lib/libANN.a" )
