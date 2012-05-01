
### Qt Gui stuff ###
SET(headers_ui 
	MainWindow.h
)

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###

SET(SRC_FILES 
	main.cpp
	${moc_srcs}
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${QT_LIBRARIES} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# create an executable file named "example-plot" from the source files
ADD_EXECUTABLE(example-plot ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(example-plot utilite utiliteqt ${LIBRARIES})