# multimap_server
Map server implementation that allows to offer multiple maps simultaneously. Based on the code of the map_server package at http://wiki.ros.org/map_server

## 1 multimap_server
Map server implementation that allows to offer multiple maps simultaneously.
You can pass a .yaml file as an argument to load an initial set of maps. An example can be found in config/multimap_server_config.yaml

### 1.1 Published Topics
* map_metadata (nav_msgs/MapMetaData)
    Receive the map metadata via this latched topic.
* map (nav_msgs/OccupancyGrid)
    Receive the map via this latched topic.

### 1.2 Services
* load_map (MultimapServer/loadMapCallback)
    Load a new map using a map .yaml file. You also have to define a namespace, a desired map_name and a frame_id for the new map.

### 1.3 Bringup
rosrun multimap_server multimap_server path_to_config_yaml_file
