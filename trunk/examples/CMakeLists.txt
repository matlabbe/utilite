
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

####################################
# Audio example
####################################
IF(BUILD_QT AND BUILD_AUDIO)
    FIND_PACKAGE(FFTW3F)
	IF(FFTW3F_FOUND)
		SET(INCLUDE_DIRS
		    ${INCLUDE_DIRS}
		    ${PROJECT_SOURCE_DIR}/audio/include
		    ${PROJECT_SOURCE_DIR}/qt/include
			${FFTW3F_INCLUDE_DIRS}
		)
		
		INCLUDE(${QT_USE_FILE})
		
		SET(LIBRARIES
		    ${QT_LIBRARIES} 
			${FFTW3F_LIBRARIES}
		)
		
		INCLUDE_DIRECTORIES(${INCLUDE_DIRS})
		
		ADD_EXECUTABLE(example-spectrogram spectrogram.cpp)
		TARGET_LINK_LIBRARIES(example-spectrogram utilite utilite_audio utilite_qt ${LIBRARIES})
	ELSE(FFTW3F_FOUND)
		MESSAGE(STATUS "FFTW3F is not found, spectrogram example app won't be compiled...")
	ENDIF(FFTW3F_FOUND)
ENDIF(BUILD_QT AND BUILD_AUDIO)