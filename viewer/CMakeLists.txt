message("Making Spectrolab Viewer")

set(CMAKE_BUILD_TYPE "Debug")
SET( QT_USE_QTXML TRUE )
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(${QT_DEFINITIONS})
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE (${QT_USE_FILE})
include_directories(${QT_INCLUDES})

 
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}
					${CMAKE_CURRENT_LIST_DIR} 
					${CMAKE_CURRENT_LIST_DIR}/src
					${CMAKE_CURRENT_LIST_DIR}/include )

SET(viewer_SOURCES src/main.cpp
				   src/cloud_player_widget.cpp
				   src/viewer_application.cpp
				   src/spectroscan_settings_widget.cpp
				   src/cloud_renderer.cpp)

SET(viewer_HEADERS include/pcl/visualization/cloud_player_widget.h
				   include/pcl/visualization/cloud_renderer.h
				   src/viewer_application.h
				   src/spectroscan_settings_widget.h
				   )

QT4_WRAP_UI(viewer_FORM_HEADERS  ui/cloud_player.ui 
								 ui/viewer_main.ui
								 ui/spectroscan_settings.ui)
QT4_ADD_RESOURCES(viewer_RESOURCES_RCC  
								   resources/viewer.qrc)


QT4_WRAP_CPP(viewer_HEADERS_MOC ${viewer_HEADERS})


ADD_EXECUTABLE(spectrolab_viewer ${viewer_SOURCES} 
								 ${viewer_HEADERS}
    						     ${viewer_HEADERS_MOC}
    							 ${viewer_FORM_HEADERS} 
    							 ${viewer_RESOURCES_RCC})
    							
TARGET_LINK_LIBRARIES(spectrolab_viewer ${QT_LIBRARIES} 
								${SPECTROLAB_DEP_LIBRARIES} 
								spectrolab pcl_spectrolab_io QVTK )


