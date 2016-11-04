project(teoMWI)


# source files
aux_source_directory(. teoMWI_SRCS)
#INCLUDE_DIRECTORIES(.)

find_package(YARP REQUIRED)

FILE(GLOB_RECURSE teoMWI_INCLUDE_LIST ./*.h ./*.hpp)


ADD_LIBRARY(${PROJECT_NAME} STATIC ${teoMWI_SRCS} ${teoMWI_INCLUDE_LIST})




