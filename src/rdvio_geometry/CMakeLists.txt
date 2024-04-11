# rdvio core
target_sources(${PROJECT_NAME}
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/essential.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/homography.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lie_algebra.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stereo.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
