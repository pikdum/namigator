add_executable(MapDiag main.cpp)

target_include_directories(MapDiag PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries(MapDiag PRIVATE
    libpathfind
    utility
    RecastNavigation::Recast
    RecastNavigation::Detour
    ${FILESYSTEM_LIBRARY}
    Threads::Threads)
