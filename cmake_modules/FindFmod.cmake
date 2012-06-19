# - Find Fmod
# This module finds an installed Fmod package.
#
# It sets the following variables:
#  Fmod_FOUND       - Set to false, or undefined, if Fmod isn't found.
#  Fmod_INCLUDE_DIRS - The Fmod include directory.
#  Fmod_LIBRARIES    - The Fmod library to link against.
#
# 
     

     
FIND_PATH(Fmod_INCLUDE_DIRS 
          fmod.h
          PATH_SUFFIXES fmodex)

FIND_LIBRARY(Fmod_LIBRARIES NAMES fmodex64 fmodexL64 fmodex)

IF (Fmod_INCLUDE_DIRS AND Fmod_LIBRARIES)
   SET(Fmod_FOUND TRUE)
ENDIF (Fmod_INCLUDE_DIRS AND Fmod_LIBRARIES)

IF (Fmod_FOUND)
   # show which Fmod was found only if not quiet
   IF (NOT Fmod_FIND_QUIETLY)
      MESSAGE(STATUS "Found Fmod: ${Fmod_LIBRARIES}")
   ENDIF (NOT Fmod_FIND_QUIETLY)
ELSE (Fmod_FOUND)
   # fatal error if Fmod is required but not found
   IF (Fmod_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Fmod... On linux, make sure that installed libraries have their symbolic links like libfmodex.so -> libfmodex-4.40.05.so (or other version).")
   ENDIF (Fmod_FIND_REQUIRED)
ENDIF (Fmod_FOUND)

