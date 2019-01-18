# add_subdirectory(include)
# set(SUB_LIBS include)
aux_source_directory(. DIR_LIB_SRCS)
add_library(camera ${DIR_LIB_SRCS})
