cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(spectrolab)
	
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

include_directories( include)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")
######################### External Libraries ###########################
	
find_package(PCL 1.6 REQUIRED COMPONENTS  io filters visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


FIND_PACKAGE( Boost 1.48 COMPONENTS 
					program_options filesystem thread
					serialization system 
					REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( include)

SET( QT_USE_QTXML TRUE )
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(${QT_DEFINITIONS})
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE (${QT_USE_FILE})
include_directories(${QT_INCLUDES})

######################### Project Variables ############################
	
set(SPECTROLAB_DEP_LIBRARIES 
	 ${PCL_LIBRARIES}
	 ${Boost_LIBRARIES}
	 ${QT_LIBRARIES}
)

######################## Executables / Libraries ######################

#add a directory for throw away programs for testing out code
if( IS_DIRECTORY "${CMAKE_SOURCE_DIR}/scratch")
message(STATUS "Adding scratch directory")
add_subdirectory(scratch)
endif()


#Project specific stuff
add_library(spectrolab src/spectroscan_3d.cpp)
target_link_libraries( spectrolab  ${Boost_LIBRARIES} )

add_library(pcl_spectrolab_io  src/spectroscan_3d_io.cpp)
target_link_libraries( pcl_spectrolab_io 
							${SPECTROLAB_DEP_LIBRARIES} 
							spectrolab)


add_subdirectory(tools)
add_subdirectory(viewer)


##################### Installation $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
INSTALL(TARGETS  spectrolab pcl_spectrolab_io
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY include/spectrolab  DESTINATION include)

	
