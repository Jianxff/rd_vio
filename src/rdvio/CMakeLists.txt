# rdvio core
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/handler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/initializer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sliding_window_tracker.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/config.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
