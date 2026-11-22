# Design: Reload loaded maps

- **Status:** Implemented
- **Requirements:** `requirements.md`
- **Plan:** `plan.md`

## Context

`MultimapServer::loadMapCallback` currently returns `success=false` when the requested `(ns, map_name)` is already loaded. `loadEnvironmentsFromYAML` skips an already-loaded identity; its duplicate handling can also skip adding other maps and report a misleading success. `Map` currently loads map data and advertises ROS endpoints in one constructor, and several map-file validation failures call `exit(-1)`.

The `load_map` service and `load_environments` service schemas are in the separate `multimap_server_msgs` package. This feature does not require changing those schemas.

## Proposed design

### Per-map replacement

Separate candidate map loading from ROS endpoint registration/publication. Construct and validate the replacement before changing the current map. If candidate loading fails, return an error to the service callback and leave the currently registered map and its published data intact. Convert service-triggered map-file errors that currently terminate the process into recoverable exceptions; initial startup failures may continue to fail startup through the existing startup path.

After a candidate is valid, unregister/destroy the old map, install the candidate at the same `(ns, map_name)` identity, and advertise/publish the replacement on the existing service and latched topics. Keep one map entry and one environment map-name entry for that identity.

### Environment YAML merge

Process each listed map independently:

- Replace an existing map with the same environment namespace and map name.
- Add a listed map that is not already loaded and add its name to the existing environment exactly once.
- Retain loaded maps in that environment that are not listed in the YAML.
- Preserve the existing environment metadata, including its `global_frame`.
- Use the incoming YAML `global_frame` for each listed map that is added or replaced.

If one or more map operations fail, return `success=false` with an error message. Keep successful operations applied, but retain the prior data for each failed replacement. This is per-map atomicity, not all-or-nothing transactionality for the entire YAML file.

## Consequential decisions

| Decision | Alternatives considered | Chosen option and rationale | Consequences / compatibility |
|---|---|---|---|
| Identity of a duplicate | Match by file path; match by `(ns, map_name)` | Use `(ns, map_name)`, matching the package's existing identity check and ROS endpoint names. | Different files can replace the same named map. |
| Failed replacement | Delete old first; retain old until candidate is loaded | Retain old until the candidate data loads successfully, as requested. | Service-triggered map-load failures must be recoverable rather than terminating the node. |
| Environment YAML collision | Replace entire environment; replace matching maps only | Replace matches, add new listed maps, and retain omitted maps. | Existing environments can be extended/refreshed without deleting unrelated maps. |
| Incoming frame differs | Change environment and all maps; ignore request; apply to replaced map only | Apply it to the added/replaced map only and leave environment metadata/other maps unchanged, as selected. | A frame mismatch is possible; preserve the existing warning/message behavior where applicable and document this consequence. |
| Partial YAML failure | Roll back all previous operations; keep successful per-map operations | Keep successful operations, report overall failure, and retain the prior data for each failed replacement. | The environment request is not an all-or-nothing transaction. |

## Error handling and edge cases

- Do not advertise a second publisher/service for an identity while the old map is still active; prepare replacement data before switching endpoints.
- YAML parse, required-field, and image-load errors during service calls must reach the service response and must not terminate the node.
- A new map in an existing environment must be added to both the map collection and the environment's `map_name` list.
- Keep the existing service and message types unchanged.

## Testing and documentation

- **Focused tests:** ROS integration tests for duplicate replacement via both services, environment merge/list consistency, and failed replacement preserving the old map. Approval to add `rostest` registration and test dependencies was received before implementation.
- **Documentation:** Update the `load_map` and `load_environments` sections in `README.md`, including the selected frame behavior.
- **Validation:** Build `multimap_server` in the catkin workspace and run the focused ROS integration tests. Exact commands and results will be recorded after implementation.

## Open questions

- None. The selected frame behavior and per-map partial-failure semantics are recorded above.
