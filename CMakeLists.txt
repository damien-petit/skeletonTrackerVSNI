### CMake file for plugin_skeletonTrackerVSNI
###
###    From the off-tree build directory, invoke:
###	 cmake <PATH_TO_vision_server>
###
###
###   - May 2012: Initial version
###
###
###############################################################################


cmake_minimum_required (VERSION 2.6)

project (skeletonTrackerVSNI)

set( CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /lib /usr/lib /usr/local/lib )
set( CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr/include /usr/local/include )

set ( INSTALL_BIN_PATH bin )
set ( INSTALL_LIB_PATH lib )
set ( INSTALL_AR_PATH  lib )
set ( INSTALL_HEADERS_PATH include )
#define a macro to simplify adding tests, then use it
# Make relative paths absolute (needed later on)

foreach(p LIB BIN HEADERS AR)
        set(var INSTALL_${p}_PATH)
        if(NOT IS_ABSOLUTE "${${var}}")
                set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
        endif()
endforeach()

### Search for Lib/Headers

find_package( vision )
find_package( visionsystem )
find_package( Boost 1.40.0 COMPONENTS system thread filesystem)
FIND_PACKAGE( OpenCV REQUIRED )

### Find OpenNI
#add a hint so that it can find it without the pkg-config
find_path(OPENNI_INCLUDE_DIR XnStatus.h
#          HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} /usr/include/openni /usr/include/ni
          HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} /usr/include/ni /usr/include/openni
          PATHS "$ENV{PROGRAMFILES}/OpenNI/Include" "$ENV{PROGRAMW6432}/OpenNI/Include"
#          PATH_SUFFIXES openni)
         )

#add a hint so that it can find it without the pkg-config
find_library(OPENNI_LIBRARY
             NAMES OpenNI64 OpenNI
             HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} /usr/lib
             PATHS "$ENV{PROGRAMFILES}/OpenNI/Lib${OPENNI_SUFFIX}" "$ENV{PROGRAMW6432}/OpenNI/Lib${OPENNI_SUFFIX}"
             PATH_SUFFIXES lib
            )

# Load Pkgconfig macros
INCLUDE(FindPkgConfig)
SET(PACKAGE_REQUIREMENTS "")

#
# Dependency on coshell-client
#
pkg_check_modules(COSHELLCLIENT coshell-client)

IF(COSHELLCLIENT_FOUND)
ADD_DEFINITIONS(-DWITH_COSHELL)
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS coshell-client --cflags
  OUTPUT_VARIABLE COSHELLCLIENT_CFLAGS)
MESSAGE(STATUS "COSHELLCLIENT_CFLAGS: ${COSHELLCLIENT_CFLAGS}")
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS coshell-client --libs
  OUTPUT_VARIABLE COSHELLCLIENT_LIBS)
MESSAGE(STATUS "COSHELLCLIENT_LIBS: ${COSHELLCLIENT_LIBS}")
ENDIF(COSHELLCLIENT_FOUND)

### Set include directories 
#specify the headers of the extern libraries that we want to use        
include_directories( ${Boost_INCLUDE_DIRS}
	                "${CMAKE_CURRENT_BINARY_DIR}"
		            ${VISION_INCLUDE_DIRS} 
		            ${VISIONSYSTEM_INCLUDE_DIRS} 
                    "${VISIONSYSTEM_INCLUDE_DIRS}/vs_controllers/openni/"
             	    "${CMAKE_CURRENT_SOURCE_DIR}/include"
                    )

include_directories(${OPENNI_INCLUDE_DIR})
message(STATUS "OpenNI found (include: ${OPENNI_INCLUDE_DIR}, lib: ${OPENNI_LIBRARY})")

### File Lists

file( GLOB openni_src "src/*.cpp" )
file( GLOB openni_hdr "src/*.h*")

#		     ${DL_INCLUDE_DIR} )


#specify the binary files of the extern libraries that we want to link to
link_directories( "${VISIONSYSTEM_LIBRARY_DIRS}" "${VISION_LIBRARY_DIR}" ${INSTALL_LIB_PATH}/vs_plugins ${INSTALL_LIB_PATH}/vs_controllers "$ENV{HOME}/devel/lib" "${OPENNI_LIBRARY_DIR}")

### Go to subdirs

SET( skeletonTrackerVSNI_lib_src 
src/skeletonTrackerVSNI.cpp
)

SET( skeletonTrackerVSNI_lib_hdr
include/skeletonTrackerVSNI.h
include/NISkeletonType.h
)

#add library with option SHARED means compile the library dynamically
add_library( "skeletonTrackerVSNI" SHARED ${skeletonTrackerVSNI_lib_src} ${skeletonTrackerVSNI_lib_hdr} )

#link the extern libraries to the library that we want to compile
target_link_libraries( "skeletonTrackerVSNI" ${VISIONSYSTEM_LIBRARIES}
				     ${VISION_LIBRARIES} 
				     ${OpenCV_LIBS} 
                     ${COSHELLCLIENT_LIBS}
                     GL configparser
			 		 XmlRpc xmlrpc-server
					 socket
                     ${OPENNI_LIBRARY}
#                     coshell-client
		             )



#SET( build_voc_src
#src/main.cpp
#)

#add_executable( "build_voc" ${build_voc_src} )

#target_link_libraries( "build_voc" recognition ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

#SET( build_masks_src
#src/build_masks.cpp
#)

#add_executable( "build_masks" ${build_masks_src} )

#target_link_libraries( "build_masks" recognition ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} sfml-graphics sfml-window sfml-system )

#SET( test_src
#test/test.cpp
#)

#add_executable( "test_reco" ${test_src} )

#target_link_libraries( "test_reco" recognition ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} XmlRpc configparser xmlrpc-server)

### INSTAll

install( TARGETS "skeletonTrackerVSNI" LIBRARY DESTINATION
     "${INSTALL_LIB_PATH}/vs_plugins"  COMPONENT main )

#install( TARGETS "build_voc"
#        RUNTIME DESTINATION bin COMPONENT main)

#install( TARGETS "build_masks"
#        RUNTIME DESTINATION bin COMPONENT main)

#install( TARGETS "test_reco"
#       RUNTIME DESTINATION ${CMAKE_BINARY_DIR}/test COMPONENT main)

#install( FILES test/test.bin data/recognition.voc
#        DESTINATION ${CMAKE_BINARY_DIR}/test COMPONENT main)

#install( FILES ${skeletonTrackerVSNI_lib_hdr} 
#         DESTINATION "${INSTALL_HEADERS_PATH}/skeletonTrackerVSNI/" COMPONENT main )
