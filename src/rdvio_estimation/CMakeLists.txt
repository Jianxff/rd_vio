# rdvio estimation
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/preintegrator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/solver.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
