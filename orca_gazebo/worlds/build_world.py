#!/usr/bin/env python

"""Build orca.world and orca_map.yaml from a list of markers and poses"""


def build_world(name, markers):
    world = open(name, 'w')
    world.write("""<?xml version="1.0"?>

<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://sun</uri>
    </include>

""")
    for marker in markers:
        world.write(f"""    <model name="marker{marker[0]}">
      <include>
        <static>true</static>
        <uri>model://marker_{marker[0]}</uri>
      </include>
      <pose>{marker[1]} {marker[2]} {marker[3]} 0 0 0</pose>
    </model>
""")
    world.write("""  </world>
</sdf>""")
    world.close()


def build_map(name, markers):
    map = open(name, 'w')
    map.write("""# Map for orca.world
# All marker locations are fixed (f: 1)

marker_length: 0.1778
markers:
""")
    for marker in markers:
        map.write(f"""  - id: {marker[0]}
    u: 1
    f: 1
    xyz: [{marker[1]}, {marker[2]}, {marker[3]}]
    rpy: [0, 0, -1.57]
""")
    map.close()


# Large pool: 8m x 8m x 4m deep
large_pool = [
    [0, 2, 0, -4],
    [1, 2, 2, -4],
    [2, 2, 4, -4],
    [3, 2, 6, -4],

    [4, 0, 6, -4],
    [5, 0, 4, -4],
    [6, 0, 2, -4],
    [7, 0, 0, -4],

    [8, -2, 0, -4],
    [9, -2, 2, -4],
    [10, -2, 4, -4],
    [11, -2, 6, -4],

    [12, -4, 6, -4],
    [13, -4, 4, -4],
    [14, -4, 2, -4],
    [15, -4, 0, -4],
]

# Small pool: 4m diameter x 1m deep
small_pool = [
    [0, 1, 0, -1],
    [1, 1, 1, -1],
    [2, 1, 2, -1],
    [3, 1, 3, -1],

    [4, 0, 3, -1],
    [5, 0, 2, -1],
    [6, 0, 1, -1],
    [7, 0, 0, -1],

    [8, -1, 0, -1],
    [9, -1, 1, -1],
    [10, -1, 2, -1],
    [11, -1, 3, -1],

    [12, -2, 3, -1],
    [13, -2, 2, -1],
    [14, -2, 1, -1],
    [15, -2, 0, -1],
]

worlds = [
    ['large.world', 'large_map.yaml', large_pool],
    ['small.world', 'small_map.yaml', small_pool],
]

for world in worlds:
    build_world(world[0], world[2])
    build_map(world[1], world[2])
