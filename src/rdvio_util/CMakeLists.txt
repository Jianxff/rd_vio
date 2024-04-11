# rdvio utils
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/debug.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/worker.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
