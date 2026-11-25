# multimap_server
Map server implementation based on the map_server package at http://wiki.ros.org/map_server that extends the functionality of the original.

## 1 multimap_server
Map server implementation that allows to offer multiple maps simultaneously.
You can pass a .yaml file as an argument to load an initial set of maps. An example can be found in config/multimap_server_config.yaml

### 1.1 Published Topics
* map_metadata (nav_msgs/MapMetaData)

    Receive the map metadata via this latched topic. One for each map.
* map (nav_msgs/OccupancyGrid)

    Receive the map via this latched topic. One for each map.
* environments (multimap_server_msgs/Environments)

    Contains information about the currently loaded environments.

### 1.2 Services
* load_environments (multimap_server_msgs/LoadEnvironments)

    Load or merge maps from an environment .yaml file. Maps with an existing namespace and name are reloaded; new maps are added. Maps already loaded in that environment but not listed in the file remain loaded.

    If any map cannot be loaded, the service returns `success: false`. Other map operations in the same file may still have succeeded, while a failed replacement leaves the previous map loaded.

    - **environments_url**: Path to environment .yaml file to be loaded

    Example:
    ```
    rosservice call /load_environments "environments_url: '/home/rb1/maps/robotnik_environments.yaml'"
    ```

* load_map (multimap_server_msgs/LoadMap)

    Load or reload a map using a map .yaml file. You also have to define a namespace, a desired map_name and a frame_id for the map. Reloading replaces the map with the same namespace and name; if loading the replacement fails, the previous map remains available.

    - **map_url**: Path to map .yaml file to be loaded
    - **ns**: Namespace in which the map will be loaded. If it matches with an existing environment, the map will be loaded on that environment. Otherwise, a new environment will be created.
    - **map_name**: Desired name for the published map
    - **global_frame**: frame_id in which the added or reloaded map will be published. If the environment already exists, its published `global_frame` and maps that are not replaced remain unchanged; a mismatch is reported as a warning.

    Example:
    ```
    rosservice call /load_map "map_url: '/home/rb1/maps/robotnik_routes_map.yaml' ns: 'robotnik_floor_0' map_name: 'routes' global_frame: 'level_0_map'"
    ```

* dump_environments (std_srvs/Trigger)

    Unloads all environments and maps.

* dump_map (multimap_server_msgs/DumpMap)

    Unloads a map from an environment.

    - **ns**: Name of the environment in which the map is loaded. This is the same as the map namespace.
    - **map_name**: Name of the map to be unloaded.

    Example:
    ```
    rosservice call /dump_map "ns: 'robotnik_floor_1' map_name: 'localization'"
    ```


### 1.3 Bringup
rosrun multimap_server multimap_server (path_to_environments_yaml_file)



## 2 online_map_saver
Map saver implementation that runs continuously and offers a service to save maps on demand.

### 2.1 Services
* save_map (multimap_server_msgs/SaveMap)
    Save a map by specifying the static_map/dynamic_map service associated to it.

    - **map_service**: Service offered (nav_msgs/GetMap) or Topic published (nav_msgs/OccupancyGrid) by the map publisher/creator to retrieve the map
    - **map_filename**: Desired name for the saved map:
      - If just a name is given, the map will be saved in the current working directory
      - If an absoulte path is given, the map will be saved there as long as the path exists and it can be modified by the user.
    - **use_default_thresholds**: If true, the default thresholds (free = 0, occ = 100) will be used. Otherwise, the values will be taken from the fields threshold_occupied and threshold_free.
    - **threshold_occupied**: Value between 1 and 100
    - **threshold_occupied**: Value between 0 and 100

    Examples:
    ```
    rosservice call /save_map "{map_service: '/floor_0/localization/static_map', map_filename: 'localization_map_0', use_default_thresholds: true, threshold_occupied: 0.0, threshold_free: 0.0}"
    ```
    ```
    rosservice call /save_map "{map_service: '/gmapping/dynamic_map', map_filename: '/home/robot/maps/robotnik_warehouse_map', use_default_thresholds: true, threshold_occupied: 0.0, threshold_free: 0.0}"
    ```
    ```
    rosservice call /save_map "{map_service: '/gmapping/map', map_filename: '/home/robot/maps/robotnik_warehouse_map', use_default_thresholds: true, threshold_occupied: 0.0, threshold_free: 0.0}"
    ```


### 2.2 Bringup
rosrun multimap_server online_map_saver

## 3 Feature development workflow

For AI-assisted feature work, see the package [agent guide](AGENTS.md), the [requirements](docs/feature-workflow/templates/requirements.md), [design](docs/feature-workflow/templates/design.md), and [plan](docs/feature-workflow/templates/plan.md) templates, and the reusable [planning prompt](.github/prompts/plan-feature.prompt.md) and [implementation prompt](.github/prompts/implement-feature.prompt.md).
