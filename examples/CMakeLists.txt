
SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

# Make sure the compiler can find include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binaries
# The extension is automatically found.
ADD_EXECUTABLE(simpleThreadExample simpleThreadExample.cpp)
TARGET_LINK_LIBRARIES(simpleThreadExample utilite)

SET_TARGET_PROPERTIES(
simpleThreadExample 
PROPERTIES
 DEBUG_POSTFIX d
)