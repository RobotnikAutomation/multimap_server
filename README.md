# multimap_server
Map server implementation based on the map_server package at http://wiki.ros.org/map_server that extends the functionality of the original.

## 1 multimap_server
Map server implementation that allows to offer multiple maps simultaneously.
You can pass a .yaml file as an argument to load an initial set of maps. An example can be found in config/multimap_server_config.yaml

### 1.1 Published Topics
* map_metadata (nav_msgs/MapMetaData)
    Receive the map metadata via this latched topic.
* map (nav_msgs/OccupancyGrid)
    Receive the map via this latched topic.

### 1.2 Services
* load_map (multimap_server_msgs/LoadMap)
    Load a new map using a map .yaml file. You also have to define a namespace, a desired map_name and a frame_id for the new map.

    - **map_url**: Path to map .yaml file to be loaded:
    - **ns**: Namespace in which the map will be loaded
    - **map_name**: Desired name for the published map
    - **global_frame**: frame_id in which the map will be published

    Example:
    ```
    rosservice call /load_map "map_url: '/home/rb1/maps/rbk_routes_map.yaml' ns: 'robotnik' map_name: 'routes' global_frame: 'routes_map'" 
    ```

### 1.3 Bringup
rosrun multimap_server multimap_server path_to_config_yaml_file



## 2 online_map_saver
Map saver implementation that runs continuously and offers a service to save maps on demand.

### 2.1 Services
* save_map (multimap_server_msgs/SaveMap)
    Save a map by specifying the static_map/dynamic_map service associated to it.

    - **map_service**: Service offered by the map publisher/creator to retrieve the map
    - **map_filename**: Desired name for the saved map:
      - If a name is given, the map will be saved in the current working directory
      - If an absoulte path is given, the map will be saved there as long as the path exists and it can be modified by the user.
    - **use_default_thresholds**: If true, the default thresholds (free = 0, occ = 100) will be used. Otherwise, the values will be taken from the fields threshold_occupied and threshold_free.
    - **threshold_occupied**: Value between 1 and 100
    - **threshold_occupied**: Value between 0 and 100

    Examples:
    ```
    rosservice call /save_map "{map_service: '/floor_0/localization/static_map', map_filename: 'localization_map_0', use_default_thresholds: true, threshold_occupied: 0.0, threshold_free: 0.0}"
    ```
    ```
    rosservice call /save_map "{map_service: '/gmapping/dynamic_map', map_filename: '/home/rb1/maps/robotnik_warehouse_map', use_default_thresholds: true, threshold_occupied: 0.0, threshold_free: 0.0}"
    ```


### 2.2 Bringup
rosrun multimap_server online_map_saver
