
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../include
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "resource_tool" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(uresourcegenerator ${SRC_FILES})
TARGET_LINK_LIBRARIES(uresourcegenerator utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
uresourcegenerator 
PROPERTIES
 DEBUG_POSTFIX ${UTILITE_DEBUG_POSTFIX}
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
)

INSTALL(TARGETS uresourcegenerator
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)