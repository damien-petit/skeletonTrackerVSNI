### CMake file for plugin_skeletonTrackerVSNI
###
###    From the off-tree build directory, invoke:
###  cmake <PATH_TO_vision_server>
###
###
###   - May 2012: Initial version
###
###
###############################################################################


cmake_minimum_required (VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME skeletonTrackerVSNI)
SET(PROJECT_CONTACT )
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "")

SET(CXX_DISABLE_WERROR True)
SET(CXX_DISABLE_WALL True)

SETUP_PROJECT()

### Search for Lib/Headers

#find_package( vision )
#find_package( visionsystem )
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
### End find OpenNI

# Dependency on coshell-client
ADD_REQUIRED_DEPENDENCY("visionsystem")
ADD_REQUIRED_DEPENDENCY("visionsystem_xmlrpc_server")
ADD_REQUIRED_DEPENDENCY("visionsystem_socket")
ADD_OPTIONAL_DEPENDENCY("coshell-client")

### Set include directories 
#specify the headers of the extern libraries that we want to use        
include_directories( 
                        ${Boost_INCLUDE_DIRS}
                        ${VISIONSYSTEM_INCLUDE_DIRS}/vs_controllers/openni/
                        "${CMAKE_CURRENT_SOURCE_DIR}/include"
                   )

include_directories(${OPENNI_INCLUDE_DIR})
message(STATUS "OpenNI found (include: ${OPENNI_INCLUDE_DIR}, lib: ${OPENNI_LIBRARY})")

### File Lists

file( GLOB openni_src "src/*.cpp" )
file( GLOB openni_hdr "src/*.h*")


#specify the binary files of the extern libraries that we want to link to
link_directories( 
#                    ${VISIONSYSTEM_LIBRARY_DIRS}/vs_controllers
#                    "${VISION_LIBRARY_DIR}" 
                    "${OPENNI_LIBRARY_DIR}"
                )

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
PKG_CONFIG_USE_DEPENDENCY(skeletonTrackerVSNI visionsystem)
PKG_CONFIG_USE_DEPENDENCY(skeletonTrackerVSNI visionsystem_xmlrpc_server)
PKG_CONFIG_USE_DEPENDENCY(skeletonTrackerVSNI visionsystem_socket)
target_link_libraries( "skeletonTrackerVSNI" 
 #                       ${VISIONSYSTEM_LIBRARIES}
 #                       ${VISION_LIBRARIES} 
                        ${OpenCV_LIBS} 
                        GL 
 #                       configparser
 #                       XmlRpc 
 #                       xmlrpc-server
 #                        socket
                        ${OPENNI_LIBRARY}
                     )

IF(COSHELL_CLIENT_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(skeletonTrackerVSNI coshell_client)
ENDIF(COSHELL_CLIENT_FOUND)


### INSTAll

install( TARGETS "skeletonTrackerVSNI"
	 LIBRARY DESTINATION lib/vs_plugins
	 COMPONENT main )

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

