# Top-Level CmakeLists.txt
cmake_minimum_required(VERSION 2.6)
PROJECT( utilite )

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")

#######################
# VERSION
#######################
SET(UTILITE_VERSION "0.2.14")

STRING(REGEX MATCHALL "[0-9]+" UTILITE_VERSION_PARTS "${UTILITE_VERSION}")

LIST(GET UTILITE_VERSION_PARTS 0 UTILITE_VERSION_MAJOR)
LIST(GET UTILITE_VERSION_PARTS 1 UTILITE_VERSION_MINOR)
LIST(GET UTILITE_VERSION_PARTS 2 UTILITE_VERSION_PATCH)

SET(UTILITE_SOVERSION "${UTILITE_VERSION_MAJOR}.${UTILITE_VERSION_MINOR}")

CONFIGURE_FILE(FindUtiLite.cmake.in FindUtiLite.cmake @ONLY)

#######################
# COMPILATION PARAMS
#######################
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
IF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "")
ENDIF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")

SET(CMAKE_DEBUG_POSTFIX "d")

ADD_DEFINITIONS( "-Wall" )
IF(WIN32 AND NOT MINGW)
    ADD_DEFINITIONS("-wd4100 -wd4512 -wd4548 -wd4619 -wd4625 -wd4626 -wd4668 -wd4710 -wd4820" -DUTILITE_VERSION="${UTILITE_VERSION}")
ELSE ()
	ADD_DEFINITIONS("-Wno-unknown-pragmas" -DUTILITE_VERSION="${UTILITE_VERSION}")
ENDIF()
#ADD_DEFINITIONS("-DUNICODE") # to test with UNICODE projects

# It is a bit messed up on Windows between STATIC and SHARED, if we don't want
# to add UtiLite_DEFINITIONS in FindUtiLite.cmake... to know from 3rd party application if it needs to
# link dynamically [UTILITE_EXP __declspec( dllimport )] or statically (UTILITE_EXP "nothing").
OPTION( BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON )

IF(MINGW)
    # Hide the --enable-auto-import warning
	SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-auto-import")  
	SET(CMAKE_MODULE_LINKER_FLAGS "-Wl,--enable-auto-import") 
	SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-auto-import") 
ENDIF(MINGW)

# GCC 4 required
IF(UNIX OR MINGW)
    EXEC_PROGRAM( gcc ARGS "-dumpversion" OUTPUT_VARIABLE GCC_VERSION )
    IF(GCC_VERSION VERSION_LESS "4.0.0")
        MESSAGE(FATAL_ERROR "GCC ${GCC_VERSION} found, but version 4.x.x minimum is required")
    ENDIF(GCC_VERSION VERSION_LESS "4.0.0")
ENDIF(UNIX OR MINGW)

#The CDT Error Parser cannot handle error messages that span 
#more than one line, which is the default gcc behavior. 
#In order to force gcc to generate single line error messages with no line wrapping
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# [Eclipse] Automatic Discovery of Include directories (Optional, but handy) 
#SET(CMAKE_VERBOSE_MAKEFILE ON)

####### SET RPATH #########
# When RPATH is activated (supported on most UNIX systems), 
# the user doesn't need to change LD_LIBRARY_PATH

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#######################
# Uninstall target, for "make uninstall"
#######################
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

#######################
# OUTPUT DIR
#######################
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#######################
# BUILD OPTIONS
#######################
OPTION(BUILD_QT "Set to ON to build Qt module" OFF)
OPTION(BUILD_AUDIO "Set to ON to build audio module" OFF)
OPTION(BUILD_TESTS "Set to ON to build tests" OFF)
OPTION(BUILD_EXAMPLES "Set to ON to build examples" ON)

#######################
# DEPENDENCIES
#######################
SET(CMAKE_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/3rdParty/include;${CMAKE_INCLUDE_PATH}")
SET(CMAKE_LIBRARY_PATH "${PROJECT_SOURCE_DIR}/3rdParty/lib;${CMAKE_LIBRARY_PATH}")

IF(UNIX)
   FIND_PACKAGE(Pthreads REQUIRED)
ENDIF(UNIX)

#######################
# SOURCES
#######################
ADD_SUBDIRECTORY( src )

IF(BUILD_QT)
    FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
	ADD_SUBDIRECTORY( qt )
	ADD_SUBDIRECTORY( uplot )
ENDIF()

IF(BUILD_AUDIO)
    FIND_PACKAGE(Fmodex REQUIRED)
    FIND_PACKAGE(Lame)
	IF(NOT Lame_FOUND)
		MESSAGE(STATUS "Lame is not found, mp3 encoder support for audio lib won't be compiled...")
	ENDIF(NOT Lame_FOUND)
	ADD_SUBDIRECTORY( audio )
ENDIF()

IF(BUILD_TESTS)
	FIND_PACKAGE(CppUnit REQUIRED)
	ADD_SUBDIRECTORY( tests )
	IF(BUILD_AUDIO)
		ADD_SUBDIRECTORY( audio/tests )
	ENDIF(BUILD_AUDIO)
ENDIF(BUILD_TESTS)

ADD_SUBDIRECTORY( resource_generator )

IF(BUILD_EXAMPLES)
	ADD_SUBDIRECTORY( examples )
ENDIF(BUILD_EXAMPLES)

#######################
# CPACK (Packaging)
#######################
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_NAME "UtiLite")
SET(CPACK_PACKAGE_VENDOR "UtiLite project")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "UtiLite SDK contains useful cross-platform utilities (Inter-thread communication, Logger, Timer, Resource file generator) for developing easily multi-threaded applications")

#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/Description.txt")
#SET(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.LESSER.txt")

SET(CPACK_PACKAGE_VERSION_MAJOR "${UTILITE_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${UTILITE_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${UTILITE_VERSION_PATCH}")

#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "UtiLite${UTILITE_VERSION_MAJOR}.${UTILITE_VERSION_MINOR}")

#SET(CPACK_PACKAGE_EXECUTABLES "")

#SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")

set(CPACK_SOURCE_IGNORE_FILES 
  "\\\\.svn/" 
  "${PROJECT_SOURCE_DIR}/build/[a-zA-Z0-9_]+" 
  "~$" 
  "${PROJECT_SOURCE_DIR}/bin/.*example"
  "${PROJECT_SOURCE_DIR}/bin/.*uresourcegenerator"
  "${PROJECT_SOURCE_DIR}/bin/.*lib${PROJECT_NAME}"
  "${PROJECT_SOURCE_DIR}/bin/.*[tT]est"
  "${PROJECT_SOURCE_DIR}/bin/.*Log"
  "\\\\.DS_Store"
)

# Share files
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/FindUtiLite.cmake DESTINATION share/${PROJECT_NAME} COMPONENT devel)

IF(WIN32)
  # NOTE : Dependencies are installed in ressource-generator directory
  SET(CPACK_GENERATOR "NSIS;ZIP")
  SET(CPACK_SOURCE_GENERATOR "ZIP")
  SET(CPACK_NSIS_PACKAGE_NAME "UtiLite ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  
  #SET(CPACK_NSIS_MUI_ICON "${CMAKE_SOURCE_DIR}\\\\utils\\\\utilite.ico")
  #SET(CPACK_NSIS_MUI_UNIICON "${CMAKE_SOURCE_DIR}\\\\utils\\\\utilite.ico")
  #SET(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}\\\\utils\\\\utilite.ico")
  #SET(CPACK_NSIS_INSTALLED_ICON_NAME "${CMAKE_SOURCE_DIR}\\\\utils\\\\utilite.ico")
  
  SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.code.google.com/p/utilite")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.code.google.com/p/utilite")
  SET(CPACK_NSIS_DISPLAY_NAME "UtiLite SDK")
  SET(CPACK_NSIS_CONTACT "matlabbe@gmail.com")
  
  SET(CPACK_NSIS_MODIFY_PATH ON)
  
ELSEIF(APPLE)
  SET(CPACK_GENERATOR "PackageMaker;TBZ2")
  SET(CPACK_SOURCE_GENERATOR "ZIP")
  
ELSE()
  SET(CPACK_GENERATOR "DEB;TBZ2")
  SET(CPACK_SOURCE_GENERATOR "ZIP")
  
  SET(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
  SET(CPACK_DEBIAN_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  SET(CPACK_DEBIAN_ARCHITECTURE i686)
  SET(CPACK_DEBIAN_PACKAGE_DEPENDS "")
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "matlabbe@gmail.com")
  SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "${CPACK_PACKAGE_DESCRIPTION_SUMMARY}")
  SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
  SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
  SET(CPACK_DEBIAN_PACKAGE_RECOMMENDS "")
  SET(CPACK_DEBIAN_PACKAGE_SUGGESTS "")
ENDIF()

INCLUDE(CPack)

MESSAGE(STATUS "--------------------------------------------")
MESSAGE(STATUS "Info :")
MESSAGE(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "  CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
IF(NOT WIN32)
	# see comment above for the BUILD_SHARED_LIBS option on Windows
	MESSAGE(STATUS "  BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}")
ENDIF(NOT WIN32)
MESSAGE(STATUS "  BUILD_QT = ${BUILD_QT}")
MESSAGE(STATUS "  BUILD_AUDIO = ${BUILD_AUDIO}")
MESSAGE(STATUS "  BUILD_TESTS = ${BUILD_TESTS}")
MESSAGE(STATUS "  BUILD_EXAMPLES = ${BUILD_EXAMPLES}")
MESSAGE(STATUS "--------------------------------------------")
