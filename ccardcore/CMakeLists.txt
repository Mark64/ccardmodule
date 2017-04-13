# basically the easiest way to do a subdirectory in cmake is to turn it into a static library
#  and then link to it in the parent directory's CMakeLists.txt

add_library(ccardcore STATIC i2cctl.c dsa.c magnetorquer.c)
target_link_libraries(ccardcore rt)

