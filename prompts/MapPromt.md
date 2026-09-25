Task: Map Production and /map Publication

Implement the reusable map-production component needed for Project 1.

This task is focused on creating and publishing a valid occupancy-grid-style map. Do not implement A* search or path planning yet.

The ROS-like publish/subscribe middleware from the previous task may remain in the repository and should be used for /map communication. Do not use ROS, ROS 2, or ROS client/runtime libraries.

Goal

Build a map component that can produce and publish a valid map on:

/map

The component must also be compatible with independently supplied maps. The planner implemented in a later task must not depend on the map produced by this component.

There is no requirement to implement a persistent map server.

Map publication

Your component must publish an OccupancyGrid-like payload to /map.

A valid example is:

{
  "header": {
    "frame_id": "map"
  },
  "info": {
    "resolution": 0.5,
    "width": 4,
    "height": 3,
    "origin": {
      "position": {
        "x": -1.0,
        "y": 2.0,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  },
  "data": [0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}

The required fields are:

header
header.frame_id
info.resolution
info.width
info.height
info.origin
data

The origin must have the Pose-like structure shown above.

The orientation is always identity/axis-aligned:

x = 0
y = 0
z = 0
w = 1
Map dimensions and data

A valid map must satisfy:

width is a positive integer;
height is a positive integer;
resolution is a positive number;
data contains exactly width * height integer entries;
cells are stored in row-major order.

For a cell with coordinates (x, y):

data[y * width + x]

represents that cell's occupancy value.

Do not assume a particular map size.

Do not assume a particular resolution.

Do not assume the map origin is (0, 0).

Do not assume obstacles have any particular arrangement.

Free and blocked cells

A cell is free exactly when:

0 <= occupancy < 50

A cell is blocked when:

occupancy < 0

or:

occupancy >= 50

Therefore values such as:

-1
50
100

must be treated as blocked.

The map producer should generate values consistent with these semantics.

Minimum map requirements

The map produced by this task must contain at least two 4-connected free cells.

This ensures that a basic nontrivial planning request can be made against the published map.

The map does not need to be large or sophisticated.

There is no required:

map size;
resolution;
origin;
obstacle pattern;
map file format;
map-generation algorithm.

Choose a simple map that is useful for testing the later planner.

Nonzero origins and resolution

The implementation must preserve the map metadata correctly.

In particular, application code must not assume:

resolution = 1
origin.x = 0
origin.y = 0

A map may use values such as:

resolution = 0.5
origin.x = -1.0
origin.y = 2.0

The map producer should publish the metadata exactly as defined by the generated map.

Later components must be able to interpret arbitrary valid values.

No persistent map server

Do not implement:

ROS map_server;
/get_map;
ROS map-server processes;
YAML/PGM map compatibility;
a persistent map-server architecture;
any particular external map-file format.

The purpose of this task is simply to demonstrate map production and publication.

The map may be:

constructed directly in code;
generated procedurally;
loaded from a simple project-specific source;
produced using another reasonable approach.

Choose the simplest design that satisfies the requirements.

Map replacement

The /map topic is not limited to the map produced by this task.

The autograder may:

subscribe to /map;
invoke the map-production functionality;
validate the resulting publication;
later publish a completely independent valid map on /map.

A newer valid map replaces the previously observed map.

Therefore, later planner components must treat /map as a normal topic and must process whichever valid map was most recently received.

Do not build the planner around the dimensions, origin, resolution, obstacles, or layout of the example map produced here.

External map compatibility

The map component itself does not need to control or validate every externally supplied map.

However, the overall Project 1 architecture must allow an external client to publish an independent valid /map message through the ROS-like middleware.

The later planner must be able to receive such maps without requiring changes to the map-production component.

For example, the planner must not assume:

width = 4
height = 3
resolution = 0.5
origin = (-1, 2)

Those values are only an example.

Separation of responsibilities

Keep map production separate from map interpretation and planning.

This task should provide the data needed by later components, but it should not implement:

A* search;
path planning;
start/goal selection;
path publication;
obstacle avoidance;
planner-specific heuristics.

The later planner should subscribe to /map and interpret the received map independently.

Reusability

The map representation and publication behavior should be reusable by later Project 1 components.

Later tasks should be able to add:

a map subscriber;
a coordinate/cell conversion utility;
an A* planner;
/path publication;

without rewriting the map-production logic.

Map-specific behavior should remain separate from generic middleware behavior.

Integration with the middleware

Use the ROS-like publish/subscribe layer from the previous task to publish /map.

Do not add special-case /map behavior to the generic middleware.

The middleware should continue treating /map as an ordinary topic.

The map producer is responsible for constructing the application-specific message.

Runtime requirements

The eventual Project 1 runtime must make it possible to invoke the map-production component and observe its /map publication through the existing communication system.

Startup and shutdown should remain simple and deterministic.

The component should not require a ROS installation or ROS runtime.

Acceptance criteria

The completed map component should make it possible to demonstrate that:

A map can be produced by the project.
A valid message is published on /map.
The published message contains all required fields.
width and height are positive integers.
resolution is positive.
data contains exactly width * height entries.

Cell data uses row-major indexing:

data[y * width + x]
The map contains at least two 4-connected free cells.
Free cells are defined as 0 <= occupancy < 50.
Negative occupancy values and values >= 50 are treated as blocked by map consumers.
The origin has the required Pose-like structure.
The map representation supports nonzero origins.
The map representation supports non-unit resolutions.
The map does not depend on ROS or ROS 2 libraries.
No ROS map_server, /get_map, YAML/PGM compatibility, or persistent map-server implementation is added.
The map producer does not contain A* or path-planning logic.
/map remains a normal generic middleware topic rather than being hard-coded into the communication layer.
A later valid /map publication can replace the map produced by this component.
Later planner code can operate on independently supplied valid grading maps rather than relying on the example map.
The implementation remains simple enough to serve as the map-production component for the later Project 1 planner.