# Implementation plan: Reload loaded maps

- **Status:** Complete
- **Requirements:** `requirements.md`
- **Design:** `design.md`

## Scope and criteria

Implement R1-R7 for the package's existing `load_map` and `load_environments` services. Do not change message definitions, delete maps omitted from environment YAML, or modify the environment frame when a map is replaced.

## Affected files

| File | Change | Requirement IDs |
|---|---|---|
| `src/main.cpp` | Add candidate loading/replacement behavior, recoverable service-load errors, and correct environment-list updates and failure reporting. | R1-R5 |
| `README.md` | Document duplicate replacement, environment merge semantics, and incoming-frame behavior. | R6 |
| `CMakeLists.txt`, `package.xml` | Register the approved focused ROS tests and their dependencies. | R7 |
| `test/reload_loaded_maps.test`, `test/test_reload_loaded_maps.py`, test fixtures | Exercise replacement through both services, environment metadata consistency, and failed reload preservation. | R1-R5, R7 |

## Steps

1. Add focused ROS integration-test infrastructure and fixtures.
2. Refactor map loading so candidate data is validated before ROS endpoints for an existing map are replaced.
3. Implement duplicate replacement in `load_map`, retaining the prior map on failure and applying the incoming frame to the replaced map.
4. Implement per-map replacement/addition in `load_environments`, retaining omitted maps, maintaining unique environment map names, and reporting any failed operation.
5. Update the package README to match the implemented service behavior.
6. Run focused tests and the package build; record evidence for every criterion.

## Validation

| Command or check | Purpose / criteria | Result |
|---|---|---|
| `catkin build multimap_server` from the catkin workspace root | Compile the package after the implementation. | Passed; all 4 selected packages succeeded. |
| `catkin run_tests multimap_server` from the catkin workspace root | Run the focused registered ROS regression tests. | Passed; 3 tests, 0 errors, 0 failures. |
| `catkin_test_results build/multimap_server/test_results` from the catkin workspace root | Confirm the catkin test result summary. | Passed; 4 test results, 0 errors, 0 failures. |

## Evidence and completion

Record evidence for each criterion; do not mark a criterion satisfied based only on planned work.

| ID | Status (Pending / Satisfied / Outstanding) | Supporting evidence |
|---|---|---|
| R1 | Satisfied | Integration test confirms duplicate replacement data is returned by service and published topics, with one environment map name. |
| R2 | Satisfied | Integration test confirms YAML replacement/addition, retained omitted map, and unique map names. |
| R3 | Satisfied | Integration tests confirm incoming map frame and unchanged environment/untouched-map frames. |
| R4 | Satisfied | Integration test confirms failed direct reload keeps previous map and later service calls still succeed. |
| R5 | Satisfied | Integration test confirms failed YAML replacement retains old map while successful operations are applied. |
| R6 | Satisfied | Package README documents the approved behavior. |
| R7 | Satisfied | All 3 ROS integration tests passed. |

Mark the requirement implemented only after every mandatory criterion passes. Otherwise report the outstanding mandatory criteria and verification gaps.

## Approval received

- Behavior and design were approved before implementation.
- Adding the first focused ROS integration-test setup to this package was approved.
