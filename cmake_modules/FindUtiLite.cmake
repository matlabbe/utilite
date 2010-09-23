# - Find UTILITE
# This module finds an installed UTILITE package.
#
# It sets the following variables:
#  UTILITE_FOUND              - Set to false, or undefined, if UTILITE isn't found.
#  UTILITE_INCLUDE_DIR - The UTILITE include directory.
#  UTILITE_LIBRARY           - The UTILITE library to link against.
#  URESOURCEGENERATOR_EXEC - The resource generator tool executable
#
# 
     
FIND_PATH(UTILITE_INCLUDE_DIR 
          UEventsManager.h
	  PATH_SUFFIXES UtiLite ../include/UtiLite)

FIND_LIBRARY(UTILITE_LIBRARY NAMES utilite PATH_SUFFIXES ../lib)

FIND_PROGRAM(URESOURCEGENERATOR_EXEC NAME uresourcegenerator PATHS)  

IF (UTILITE_INCLUDE_DIR AND UTILITE_LIBRARY)
   SET(UTILITE_FOUND TRUE)
ENDIF (UTILITE_INCLUDE_DIR AND UTILITE_LIBRARY)

IF (UTILITE_FOUND)
   # show which UTIL was found only if not quiet
   IF (NOT UTILITE_FIND_QUIETLY)
      MESSAGE(STATUS "Found UtiLite: ${UTILITE_LIBRARY}")
   ENDIF (NOT UTILITE_FIND_QUIETLY)
   IF (NOT URESOURCEGENERATOR_EXEC)
      MESSAGE(STATUS "uresourcegenerator was not found…")
   ENDIF (NOT URESOURCEGENERATOR_EXEC)
ELSE (UTILITE_FOUND)
   # fatal error if UTILITE is required but not found
   IF (UTILITE_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find UtiLite")
   ENDIF (UTILITE_FIND_REQUIRED)
ENDIF (UTILITE_FOUND)

