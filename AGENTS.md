# multimap_server agent guide

## Scope and ownership

- This guide applies to the `multimap_server` ROS package.
- Keep changes within this package unless a directly required change belongs elsewhere. Ask before changing shared message definitions, another package, public behavior, or otherwise expanding the agreed scope.
- Preserve existing ROS topics, services, names, and configuration behavior unless the approved requirements say otherwise.
- Do not commit, create branches, or change release versions unless asked.

## Package context

- This is a ROS 1 catkin C++ package. `CMakeLists.txt` builds `multimap_server`, `online_map_saver`, and the `multimap_server_image_loader` library.
- ROS message and service interfaces come from `multimap_server_msgs`; map loading also uses yaml-cpp, SDL/SDL_image, and Bullet.
- Follow the neighboring code and package documentation. The repository's VS Code C++ configuration specifies C++14, but the package CMake file does not enforce a C++ standard; do not treat that editor setting as a package build guarantee.
- Update the package README when user-visible topics, services, commands, or configuration behavior change. Verify referenced paths against the checked-in package contents.

## Build and tests

- The repository README documents `catkin build` from the catkin workspace root.
- The package CMake file currently declares no tests, and no package test files were found. Do not report a package test command or test suite as existing without checking for a later addition.
- For behavior changes, add focused tests using an available project test mechanism. If none exists, explain the gap and ask before adding a new test framework or expanding package scope. Report the exact commands run and any validation not performed.

## Feature workflow

1. Use the feature prompt and templates linked from the package README. Requirements must have testable acceptance criteria and identify which are mandatory.
2. Record consequential implementation decisions and rationale in the design. The plan must link to the requirements and design, name affected files, and list validation commands.
3. Before implementation, confirm the approved scope and behavior. Ask before changing either.
4. Implement the approved plan, including focused tests and relevant documentation. Do not silently omit a test; state why it could not be added or run.
5. Mark an acceptance criterion satisfied only with supporting evidence. Mark a requirement implemented only when every mandatory criterion passes; otherwise report outstanding criteria and verification gaps.
