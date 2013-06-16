add_executable(spectroscan3d_simple_viewer spectroscan3d_simple_viewer.cpp)
target_link_libraries(spectroscan3d_simple_viewer ${SPECTROLAB_DEP_LIBRARIES} 
													spectrolab pcl_spectrolab_io)

add_executable(spectroscan3d_grabpcd spectroscan3d_grabpcd.cpp)
target_link_libraries(spectroscan3d_grabpcd ${SPECTROLAB_DEP_LIBRARIES} 
													spectrolab pcl_spectrolab_io)

add_executable(spectroscan3d_grabframe spectroscan3d_grabframe.cpp)
target_link_libraries(spectroscan3d_grabframe ${SPECTROLAB_DEP_LIBRARIES} 
													spectrolab pcl_spectrolab_io)

if ( ${PCL_VERSION} GREATER 1.6)
add_executable(spectroscan3d_frame_to_png spectroscan3d_frame_to_png.cpp)
target_link_libraries(spectroscan3d_frame_to_png ${SPECTROLAB_DEP_LIBRARIES} spectrolab pcl_spectrolab_io)
endif()
				
add_executable(simple_movie_player simple_movie_player.cpp)
target_link_libraries(simple_movie_player ${SPECTROLAB_DEP_LIBRARIES} 
										 spectrolab pcl_spectrolab_io)
															