
SET(SRC_FILES
    UEventsManager.cpp
    UEventsHandler.cpp
    UEventsSender.cpp
    UFile.cpp
    UDir.cpp
    UConversion.cpp
    ULogger.cpp
    UThread.cpp
    UTimer.cpp
    UProcessInfo.cpp
    UVariant.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${PTHREADS_INCLUDE_DIR}
)

# Make sure the compiler can find include files from our UtiLite library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Adds a library called UtiLite (libutilite.so under Linux) from the source files
ADD_LIBRARY( utilite ${SRC_FILES} )
IF(MINGW)
    TARGET_LINK_LIBRARIES(utilite ${PTHREADS_LIBRARY} "-lpsapi")
ELSEIF(WIN32)
    FIND_LIBRARY(PSAPI_LIBRARIES NAMES psapi libpsapi.dll.a libpsapi.a libpsapi.lib )
    TARGET_LINK_LIBRARIES(utilite ${PTHREADS_LIBRARY} ${PSAPI_LIBRARIES})
ELSE()
    TARGET_LINK_LIBRARIES(utilite ${PTHREADS_LIBRARY})
ENDIF()

SET_TARGET_PROPERTIES(
utilite
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS utilite
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
        DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT devel
        FILES_MATCHING PATTERN "*.h" 
        PATTERN ".svn" EXCLUDE)
