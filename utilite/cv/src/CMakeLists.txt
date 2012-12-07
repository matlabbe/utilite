
SET(SRC_FILES
	
    UImageCapture.cpp
	UColorTable.cpp

	resources/IndexedTable1024.cpp
	resources/IndexedTable2048.cpp
	resources/IndexedTable4096.cpp
	resources/IndexedTable8192.cpp
	resources/IndexedTable16384.cpp
	resources/IndexedTable32768.cpp
	resources/IndexedTable65536.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBS} 
	${ZLIB_LIBRARIES}
)

####################################
# Generate resources files
####################################
SET(R
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes65536.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes32768.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes16384.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes8192.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes4096.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes2048.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes1024.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes512.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes256.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes128.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes64.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes32.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes16.bin.zip
	${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes8.bin.zip
)

#replace semicolons by spaces
foreach(arg ${R})
   set(RESOURCES "${RESOURCES}" "${arg}")
endforeach(arg ${R})

SET(RESOURCES_HEADERS 
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes65536_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes32768_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes16384_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes8192_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes4096_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes2048_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes1024_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes512_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes256_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes128_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes64_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes32_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes16_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes8_bin_zip.h
)

ADD_CUSTOM_COMMAND(
   OUTPUT ${RESOURCES_HEADERS}
   COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/uresourcegenerator -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
   COMMENT "[Creating resources]"
   DEPENDS ${R} uresourcegenerator
)

####################################
# Generate resources files END
####################################

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(utilite_cv ${SRC_FILES} ${RESOURCES_HEADERS})
TARGET_LINK_LIBRARIES(utilite_cv utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
utilite_cv
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS utilite_cv
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)

INSTALL(DIRECTORY ../include/${PROJECT_NAME} 
        DESTINATION include 
		COMPONENT devel
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
