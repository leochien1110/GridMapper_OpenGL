project (ThirdParty)

# Header only library, therefore INTERFACE keyword
add_library(thirdparty  INTERFACE)

#INTERFACE tagets only have properties
target_include_directories(thirdparty  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)