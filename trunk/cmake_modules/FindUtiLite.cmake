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
          utilite/UEventsManager.h)

FIND_LIBRARY(UTILITE_LIBRARY NAMES utilite)

FIND_PROGRAM(URESOURCEGENERATOR_EXEC NAME uresourcegenerator PATHS)  

IF (UTILITE_INCLUDE_DIR AND UTILITE_LIBRARY)
   SET(UTILITE_FOUND TRUE)
ENDIF (UTILITE_INCLUDE_DIR AND UTILITE_LIBRARY)

IF (UTILITE_FOUND)
   # show which UTIL was found only if not quiet
   IF (NOT UtiLite_FIND_QUIETLY)
      MESSAGE(STATUS "Found UtiLite: ${UTILITE_LIBRARY}")
   ENDIF (NOT UtiLite_FIND_QUIETLY)
   IF (NOT URESOURCEGENERATOR_EXEC)
      MESSAGE(STATUS "uresourcegenerator was not found")
   ENDIF (NOT URESOURCEGENERATOR_EXEC)
ELSE ()
   # fatal error if UTILITE is required but not found
   IF (UtiLite_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find UtiLite. Verify your PATH if it is already installed or download it at http://code.google.com/p/utilite/")
   ENDIF (UtiLite_FIND_REQUIRED)
ENDIF ()

