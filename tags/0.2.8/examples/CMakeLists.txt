
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

ADD_EXECUTABLE(eventsExample eventsExample.cpp)
TARGET_LINK_LIBRARIES(eventsExample utilite)

ADD_EXECUTABLE(producerConsumerExample producerConsumerExample.cpp)
TARGET_LINK_LIBRARIES(producerConsumerExample utilite)

