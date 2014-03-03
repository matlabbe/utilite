
SET(headers_ui 
	../include/utilite/UPlot.h
	../include/utilite/USpectrogram.h
	../include/utilite/UImageView.h
)

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})

SET(SRC_FILES 
	./UPlot.cpp
	./USpectrogram.cpp
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

ADD_LIBRARY(utilite_qt ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(utilite_qt utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
utilite_qt
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS utilite_qt
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
        DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT devel
        FILES_MATCHING PATTERN "*.h" 
        PATTERN ".svn" EXCLUDE)
        
		