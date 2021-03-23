cmake_minimum_required(VERSION 2.8.3)
project(multimap_server)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            tf2
            roslib
            multimap_server_msgs
        )

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        multimap_server_image_loader
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        tf2
        multimap_server_msgs
)

include_directories(
    include
    ${BULLET_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

add_library(multimap_server_image_loader src/image_loader.cpp)
add_dependencies(multimap_server_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multimap_server_image_loader
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

add_executable(multimap_server src/main.cpp)
add_dependencies(multimap_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multimap_server
    multimap_server_image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(online_map_saver src/online_map_saver.cpp)
add_dependencies(online_map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(online_map_saver
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)


## Install executables and/or libraries
install(TARGETS multimap_server multimap_server_image_loader online_map_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
