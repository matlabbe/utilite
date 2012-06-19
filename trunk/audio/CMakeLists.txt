ADD_SUBDIRECTORY( src )

IF(CPPUNIT_FOUND)
	ADD_SUBDIRECTORY( tests )
ELSE(CPPUNIT_FOUND)
   MESSAGE(STATUS "CppUnit is not found, tests for the ${PROJECT_NAME} audio project won't be compiled...")
ENDIF(CPPUNIT_FOUND)