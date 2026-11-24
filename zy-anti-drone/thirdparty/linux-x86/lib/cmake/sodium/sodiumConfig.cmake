set(sodium_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(sodium_LIBRARY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
set(sodium_LIBRARIES sodium)

include_directories(${sodium_INCLUDE_DIR})
link_directories(${sodium_LIBRARY_DIR})
