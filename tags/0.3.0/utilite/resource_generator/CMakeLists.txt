
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ../include
)

SET(LIBRARIES utilite)

########
# HACK for dependencies packaging below, give all dependencies
########
IF(BUILD_AUDIO)
	ADD_DEFINITIONS(-DBUILD_AUDIO)
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ../audio/include)
	SET(LIBRARIES ${LIBRARIES} utilite_audio)
ENDIF(BUILD_AUDIO)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "resource_tool" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(uresourcegenerator ${SRC_FILES})
TARGET_LINK_LIBRARIES(uresourcegenerator ${LIBRARIES})

SET_TARGET_PROPERTIES(
uresourcegenerator 
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
)

INSTALL(TARGETS uresourcegenerator
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
		
		
########
# Dependencies packaging
########		
IF(WIN32)
	SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/uresourcegenerator${CMAKE_EXECUTABLE_SUFFIX}")
	SET(LIBS "")
	SET(DIRS ${PROJECT_SOURCE_DIR}/bin)
	install(CODE "
		include(\"BundleUtilities\")
		fixup_bundle(\"${APPS}\" \"\${LIBS}\" \"${DIRS}\")
		" COMPONENT runtime)
ENDIF(WIN32)