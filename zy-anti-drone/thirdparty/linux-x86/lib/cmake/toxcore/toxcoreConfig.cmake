set(toxcore_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(toxcore_LIBRARY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
set(toxcore_LIBRARIES toxcore)

include_directories(${toxcore_INCLUDE_DIR})
link_directories(${toxcore_LIBRARY_DIR})
