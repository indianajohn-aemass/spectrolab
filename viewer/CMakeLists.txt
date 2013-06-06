SET( QT_USE_QTXML TRUE )
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(${QT_DEFINITIONS})
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE (${QT_USE_FILE})
include_directories(${QT_INCLUDES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

SET(viewer_SOURCES src/main.cpp)

SET(viewer_HEADERS )


QT4_WRAP_UI(viewer_FORMS_HEADERS  ui/cloud_player.ui 
								  ui/viewer_main.ui)
QT4_ADD_RESOURCES(viewer_RESOURCES_RCC  
								  resources/viewer.qrc)


QT4_WRAP_CPP(viewer_HEADERS_MOC ${viewer_HEADERS})


ADD_EXECUTABLE(spectrolab_viewer ${viewer_SOURCES} 
    							${viewer_HEADERS_MOC}
    							${helloworld_FORMS_HEADERS} 
    							${helloworld_RESOURCES_RCC})
TARGET_LINK_LIBRARIES(spectrolab_viewer ${QT_LIBRARIES} 
								${SPECTROLAB_DEP_LIBRARIES} 
								spectrolab)


