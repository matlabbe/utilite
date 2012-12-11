
SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

# Make sure the compiler can find include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binaries
# The extension is automatically found.
ADD_EXECUTABLE(example-simpleThread simpleThread.cpp)
TARGET_LINK_LIBRARIES(example-simpleThread utilite)

ADD_EXECUTABLE(example-events events.cpp)
TARGET_LINK_LIBRARIES(example-events utilite)

ADD_EXECUTABLE(example-producerConsumer producerConsumer.cpp)
TARGET_LINK_LIBRARIES(example-producerConsumer utilite)

ADD_EXECUTABLE(example-counterThread counterThread.cpp)
TARGET_LINK_LIBRARIES(example-counterThread utilite)

ADD_EXECUTABLE(example-math math.cpp)
TARGET_LINK_LIBRARIES(example-math utilite)

IF(OpenCV_FOUND)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(OpenCV_FOUND)

IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
	INCLUDE(${QT_USE_FILE})
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/qt/include)
ENDIF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)

IF(TARGET utilite_cv)
MESSAGE(STATUS "test3")
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/cv/include ${ZLIB_INCLUDE_DIRS})
	ADD_EXECUTABLE(example-colorIndexesGenerator colorIndexesGenerator.cpp)
	TARGET_LINK_LIBRARIES(example-colorIndexesGenerator utilite utilite_cv ${OpenCV_LIBS} ${ZLIB_LIBRARIES})
ENDIF(TARGET utilite_cv)

MESSAGE(STATUS "test")
IF(TARGET utilite_audio)
MESSAGE(STATUS "test2")
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/audio/include)
	ADD_EXECUTABLE(example-audioCapture audioCapture.cpp)
	TARGET_LINK_LIBRARIES(example-audioCapture utilite utilite_audio utilite_qt ${QT_LIBRARIES})
ENDIF(TARGET utilite_audio)

IF(TARGET utilite_cvqt)
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/cv_qt/include)
	ADD_EXECUTABLE(example-imageCapture imageCapture.cpp)
	TARGET_LINK_LIBRARIES(example-imageCapture utilite utilite_cv utilite_cvqt ${OpenCV_LIBS} ${QT_LIBRARIES})
ENDIF(TARGET utilite_cvqt)



####################################
# Resources example
# Note that external projects using FindUtiLite.cmake 
# would use ${URESOURCEGENERATOR_EXEC} variable to get 
# the COMMAND path. DEPENDS on uresourcegenerator 
# should not be required.
####################################
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/data_txt.h 
   COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/uresourcegenerator -n my_namespace -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/data.txt
   COMMENT "[Creating resources]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/data.txt uresourcegenerator
)
ADD_EXECUTABLE(example-resources resources.cpp ${CMAKE_CURRENT_BINARY_DIR}/data_txt.h)
TARGET_LINK_LIBRARIES(example-resources utilite)