find_package(Pangolin REQUIRED)

# EuRoC
add_executable(euroc
    ${CMAKE_CURRENT_SOURCE_DIR}/euroc.cpp
)
target_link_libraries(euroc 
    PRIVATE 
        ${PROJECT_NAME}
        ${Pangolin_LIBRARIES}
)

