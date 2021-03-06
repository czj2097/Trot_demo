cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(CMakeDemo_by_PY)


if(UNIX)
	set(CMAKE_C_COMPILER "/usr/bin/gcc")
	set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)


# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -lrt -ldl")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
if(UNIX)
include_directories("/usr/xenomai/include")
include_directories("/opt/etherlab/include")
include_directories("/usr/lapack/include")
include_directories("/usr/Aris/include")
include_directories("/usr/Robots/include")
link_directories(/usr/lapack/lib)
link_directories(/usr/Aris/lib)
link_directories(/usr/Robots/lib)
link_directories(/usr/xenomai/lib)
link_directories(/opt/etherlab/lib)
link_directories(/usr/xsens/lib)
endif(UNIX)
if(WIN32)
include_directories("C:/lapack/include")
include_directories("C:/Aris/include")
include_directories("C:/Robots/include")
link_directories(C:/lapack/lib)
link_directories(C:/Aris/lib)
link_directories(C:/Robots/lib)
link_directories(C:/xsens/lib)
endif(WIN32)


# set link files
if(WIN32)
set(SYSTEM_LINK_LIBS ws2_32)
set(LAPACK_LINK_LIBS liblapacke liblapack cblas libblas)
set(XSENS_LINK_LIBS xdeviceclass)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Sensor Aris_Core)
set(ROBOTS_LINK_LIBS HexapodIV HexapodIII Robot_Client Robot_Server Robot_Base)
endif(WIN32)
if(UNIX)
set(SYSTEM_LINK_LIBS gfortran)
set(XENOMAI_LINK_LIBS native rtdm xenomai)
set(ETHERLAB_LINK_LIBS ethercat rt)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(XSENS_LINK_LIBS xdeviceclass xcommunication xstypes)
set(ARIS_LINK_LIBS Aris_Control Aris_Plan Aris_DynKer Aris_ExpCal Aris_Sensor Aris_Core)
set(ROBOTS_LINK_LIBS HexapodIV HexapodIII Robot_Client Robot_Server Robot_Base)
endif(UNIX)

################################### build server ####################################

set(SOURCE_FILES main.cpp Gait.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})
add_executable(Server ${FULL_SRC})
target_link_libraries(Server ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${XSENS_LINK_LIBS} ${ETHERLAB_LINK_LIBS} ${XENOMAI_LINK_LIBS} ${SYSTEM_LINK_LIBS})

################################### build client ####################################
add_executable(Client Client/client.cpp)
target_link_libraries(Client ${ROBOTS_LINK_LIBS} ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})

################################### install ####################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Robots/CMakeDemo/Robot_III")
endif(UNIX)

install(TARGETS Server Client RUNTIME DESTINATION bin)


























