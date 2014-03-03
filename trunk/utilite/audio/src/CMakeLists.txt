
SET(SRC_FILES
	UAudioCaptureFile.cpp
    UAudioCaptureMic.cpp
    UAudioPlayer.cpp
    UAudioCapture.cpp
    UAudioSystem.cpp
    UAudioPlayerTone.cpp
    UAudioCaptureFFT.cpp
    UAudioEvent.cpp
    UWav.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${Fmodex_INCLUDE_DIRS}
	${FFTW3F_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${Fmodex_LIBRARIES}
    ${FFTW3F_LIBRARIES}
)

IF(Lame_FOUND)
	SET(SRC_FILES ${SRC_FILES} UMp3Encoder.cpp)
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Lame_INCLUDE_DIRS})
    SET(LIBRARIES ${LIBRARIES} ${Lame_LIBRARIES})
    ADD_DEFINITIONS(-DBUILT_WITH_LAME)
ENDIF(Lame_FOUND)

####################################
# Generate resources files END
####################################


# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(utilite_audio ${SRC_FILES})
TARGET_LINK_LIBRARIES(utilite_audio utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
utilite_audio
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS utilite_audio
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
        DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT devel
        FILES_MATCHING PATTERN "*.h" 
        PATTERN ".svn" EXCLUDE)
