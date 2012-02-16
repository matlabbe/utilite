
SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
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

