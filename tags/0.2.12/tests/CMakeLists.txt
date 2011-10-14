
SET(SRC_FILES
    main.cpp
    Tests.cpp
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${CPPUNIT_INCLUDE_DIR}
    ${PTHREADS_INCLUDE_DIR}
)

# Make sure the compiler can find include files from our UtiLite library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_DEFINITIONS(${CPPUNIT_DEFINITIONS})

# Add binary called "testUtiLite" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(testUtiLite ${SRC_FILES})
TARGET_LINK_LIBRARIES(testUtiLite utilite ${CPPUNIT_LIBRARY} ${PTHREADS_LIBRARY})

SET_TARGET_PROPERTIES(
testUtiLite
PROPERTIES
 DEBUG_POSTFIX d
)

