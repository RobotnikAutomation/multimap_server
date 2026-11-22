# Requirements: Reload loaded maps

- **Status:** Approved
- **Scope:** `localization/multimap_server`
- **Related design:** `design.md`
- **Related plan:** `plan.md`

## Problem and intended outcome

Loading a map whose `(ns, map_name)` identity is already present currently rejects or skips that map instead of refreshing its data. A caller must be able to reload that identity so the published map reflects the new map file without creating a duplicate entry.

## Scope

### In scope

- Reloading a duplicate map through `load_map`.
- Reloading matching maps and adding new maps through `load_environments`.
- Preserving the old map if its replacement cannot be loaded.
- Applying the incoming `global_frame` to the replaced map only. Do not change the existing environment metadata or maps that were not replaced.

### Out of scope

- Removing currently loaded maps omitted from an environment YAML file.
- Changing service/message definitions or reloading all maps when one map is replaced.
- Changing the environment's published `global_frame` when a map is replaced.

## Acceptance criteria

Each criterion is mandatory and must be marked **Satisfied** only with supporting test or inspection evidence.

| ID | Testable acceptance criterion | Mandatory? | Status (Pending / Satisfied / Outstanding) | Evidence (test, command, or inspected result) |
|---|---|---|---|---|
| R1 | Calling `load_map` for an already-loaded `(ns, map_name)` with a valid different map file returns `success=true`; the existing map is replaced, and its map topic, metadata topic, and `static_map` service expose the replacement data without adding a duplicate environment entry. | Yes | Satisfied | `test_load_map_replaces_duplicate_and_failed_reload_keeps_previous_map` passed; checks service, map topic, metadata topic, and environment entry. |
| R2 | Loading an environment YAML into an existing environment replaces each listed map whose `(ns, map_name)` already exists and adds each listed map that is new. If all listed operations succeed, the service returns `success=true`. Maps in that environment that are omitted from the YAML remain loaded, and each map name appears only once in the environment list. | Yes | Satisfied | `test_environment_yaml_replaces_adds_and_preserves_maps` passed. |
| R3 | A replacement map uses the incoming `global_frame`. Replacing a map does not change the existing environment's published `global_frame` or the frame/data of maps that were not replaced. | Yes | Satisfied | Both environment and direct-load integration tests passed with distinct map and environment frames. |
| R4 | If a replacement map cannot be parsed or loaded, the service returns `success=false`, the previous map remains available with its previous data, and the node remains running. | Yes | Satisfied | `test_load_map_replaces_duplicate_and_failed_reload_keeps_previous_map` passed, including a successful service call after the failed reload. |
| R5 | If any map operation in an environment YAML fails, `load_environments` returns `success=false` with an explanatory message. Each successfully loaded or replaced map remains applied; the map that failed to replace retains its previous data. | Yes | Satisfied | `test_environment_yaml_reports_failed_replacement_and_keeps_old_map` passed. |
| R6 | The package README documents duplicate-map replacement, environment YAML merge behavior, and the `global_frame` behavior without changing ROS service/message interfaces. | Yes | Satisfied | Updated package `README.md`; service definitions are unchanged. |
| R7 | Focused regression tests cover duplicate replacement, YAML environment merge behavior, and preservation of the old map after a failed replacement. | Yes | Satisfied | `catkin run_tests multimap_server`: 3 integration tests passed, 0 errors/failures. |

## Open questions and assumptions

- The selected `global_frame` behavior can make a replaced map's frame differ from the environment metadata and from maps not replaced. This is intentional for this requirement.
- The package previously had no declared tests. Approval to add a ROS integration-test setup was received before implementation.

## Completion

Mark this requirement **Implemented** only when every mandatory criterion is Satisfied with evidence. Otherwise mark it **Incomplete** and list outstanding mandatory criteria and verification gaps.

- **Requirement status:** Implemented
- **Outstanding criteria:** None
- **Verification gaps:** None for the approved acceptance criteria.
