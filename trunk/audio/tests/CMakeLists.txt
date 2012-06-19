
SET(SRC_FILES
    main.cpp
    Tests.cpp
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CPPUNIT_INCLUDE_DIR}
)

SET(LIBRARIES
    ${CPPUNIT_LIBRARY}
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_DEFINITIONS(${CPPUNIT_DEFINITIONS})
IF(Lame_FOUND)
    ADD_DEFINITIONS(-DBUILT_WITH_LAME)
ENDIF(Lame_FOUND)

# Add binary called "testAvpdCore" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(testAudio ${SRC_FILES})
TARGET_LINK_LIBRARIES(testAudio utilite_audio ${LIBRARIES})

