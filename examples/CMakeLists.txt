find_package(Pangolin REQUIRED)

# EuRoC
add_executable(test_euroc
    ${CMAKE_CURRENT_SOURCE_DIR}/test_euroc.cpp
)
target_link_libraries(test_euroc 
    PRIVATE 
        ${PROJECT_NAME}
        ${Pangolin_LIBRARIES}
)

# ADVIO
add_executable(test_advio
    ${CMAKE_CURRENT_SOURCE_DIR}/test_advio.cpp
)
target_link_libraries(test_advio 
    PRIVATE 
        ${PROJECT_NAME}
        ${Pangolin_LIBRARIES}
)

