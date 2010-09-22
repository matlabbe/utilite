
SET(SRC_FILES
    UEventsManager.cpp
    UEventsHandler.cpp
    UFile.cpp
    UDirectory.cpp
    UConversion.cpp
    ULogger.cpp
    UStateThread.cpp
    UTimer.cpp
    UProcessInfo.cpp
    UMathFunctions.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${PTHREADS_INCLUDE_DIR}
)

IF(WIN32)
    #CMake a l'air d'ajouter automatiquement -Dutilite_EXPORTS
	ADD_DEFINITIONS(-DUTILITE_EXPORTS) 
ENDIF(WIN32)

# Make sure the compiler can find include files from our UtiLite library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Adds a library called UtiLite (libutilite.so under Linux) from the source files
ADD_LIBRARY( utilite ${SRC_FILES} )
IF(WIN32)
    TARGET_LINK_LIBRARIES(utilite ${PTHREADS_LIBRARY} "-lpsapi")
ELSE(WIN32)
    TARGET_LINK_LIBRARIES(utilite ${PTHREADS_LIBRARY})
ENDIF(WIN32)

SET_TARGET_PROPERTIES(
utilite
PROPERTIES
 DEBUG_POSTFIX ${UTILITE_DEBUG_POSTFIX}
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
)

INSTALL(TARGETS utilite
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ 
        DESTINATION include/${PROJECT_NAME} 
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
