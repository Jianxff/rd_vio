# rdvio extra
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/opencv_image.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/yaml_config.cpp
        
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
