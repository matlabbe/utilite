# - Find Lame
# This module finds an installed Lame package.
#
# It sets the following variables:
#  Lame_FOUND       - Set to false, or undefined, if Lame isn't found.
#  Lame_INCLUDE_DIRS - The Lame include directory.
#  Lame_LIBRARIES    - The Lame library to link against.
#
# 
     

     
FIND_PATH(Lame_INCLUDE_DIRS 
          lame.h
          PATH_SUFFIXES lame)

FIND_LIBRARY(Lame_LIBRARIES NAMES mp3lame)

IF (Lame_INCLUDE_DIRS AND Lame_LIBRARIES)
   SET(Lame_FOUND TRUE)
ENDIF (Lame_INCLUDE_DIRS AND Lame_LIBRARIES)

IF (Lame_FOUND)
   # show which Lame was found only if not quiet
   IF (NOT Lame_FIND_QUIETLY)
      MESSAGE(STATUS "Found Lame: ${Lame_LIBRARIES}")
   ENDIF (NOT Lame_FIND_QUIETLY)
ELSE (Lame_FOUND)
   # fatal error if Lame is required but not found
   IF (Lame_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Lame (aka libmp3lame-dev)")
   ENDIF (Lame_FIND_REQUIRED)
ENDIF (Lame_FOUND)

