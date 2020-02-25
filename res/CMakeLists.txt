project (Res)

# Header only library, therefore INTERFACE keyword
add_library(res  INTERFACE)

#INTERFACE tagets only have properties
#target_include_directories(res  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)