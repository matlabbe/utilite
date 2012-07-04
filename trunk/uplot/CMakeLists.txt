
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ../include
    ../qt/include
)

SET(LIBRARIES utilite utilite_qt ${QT_LIBRARIES})

INCLUDE(${QT_USE_FILE})

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(uplot ${SRC_FILES})
TARGET_LINK_LIBRARIES(uplot ${LIBRARIES})

SET_TARGET_PROPERTIES(
uplot 
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
)

INSTALL(TARGETS uplot
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
        