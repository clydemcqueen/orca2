#!/usr/bin/env python

"""
Build Gazebo world and fiducial_vlam map files from a list of markers and poses
Usage:
    cd src/orca2/orca_gazebo/worlds
    build_world.py

Marker format: [marker_num, x, y, z, roll, pitch, yaw]
"""

import math
import transformations as xf


# SDF and fiducial_vlam have different coordinate models
xform = xf.quaternion_matrix([math.sqrt(0.5), 0, 0, -math.sqrt(0.5)])


def build_world(name, markers):
    world_file = open(name, 'w')
    world_file.write("""<?xml version="1.0"?>

<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://sun</uri>
    </include>

""")
    for marker in markers:
        world_file.write(f"""    <model name="marker{marker[0]}">
      <include>
        <static>true</static>
        <uri>model://marker_{marker[0]}</uri>
      </include>
      <pose>{marker[1]} {marker[2]} {marker[3]} {marker[4]} {marker[5]} {marker[6]}</pose>
    </model>
""")
    world_file.write("""  </world>
</sdf>""")
    world_file.close()


def build_map(name, markers):
    map_file = open(name, 'w')
    map_file.write("""# Map for orca.world
# All marker locations are fixed (f: 1)

marker_length: 0.1778
markers:
""")
    for marker in markers:
        m_world = xf.euler_matrix(marker[4], marker[5], marker[6])
        m_map = m_world @ xform
        e_map = xf.euler_from_matrix(m_map)

        map_file.write(f"""  - id: {marker[0]}
    u: 1
    f: 1
    xyz: [{marker[1]}, {marker[2]}, {marker[3]}]
    rpy: [{e_map[0]}, {e_map[1]}, {e_map[2]}]
""")
    map_file.close()


# Large pool: 8m x 8m x 4m deep
large_pool = [
    [0, 2, 0, -4, 0, 0, 0],
    [1, 2, 2, -4, 0, 0, 0],
    [2, 2, 4, -4, 0, 0, 0],
    [3, 2, 6, -4, 0, 0, 0],

    [4, 0, 6, -4, 0, 0, 0],
    [5, 0, 4, -4, 0, 0, 0],
    [6, 0, 2, -4, 0, 0, 0],
    [7, 0, 0, -4, 0, 0, 0],

    [8, -2, 0, -4, 0, 0, 0],
    [9, -2, 2, -4, 0, 0, 0],
    [10, -2, 4, -4, 0, 0, 0],
    [11, -2, 6, -4, 0, 0, 0],

    [12, -4, 6, -4, 0, 0, 0],
    [13, -4, 4, -4, 0, 0, 0],
    [14, -4, 2, -4, 0, 0, 0],
    [15, -4, 0, -4, 0, 0, 0],
]

# Small pool: 4m diameter x 1m deep
small_pool = [
    [0, 1, 0, -1, 0, 0, 0],
    [1, 1, 1, -1, 0, 0, 0],
    [2, 1, 2, -1, 0, 0, 0],
    [3, 1, 3, -1, 0, 0, 0],

    [4, 0, 3, -1, 0, 0, 0],
    [5, 0, 2, -1, 0, 0, 0],
    [6, 0, 1, -1, 0, 0, 0],
    [7, 0, 0, -1, 0, 0, 0],

    [8, -1, 0, -1, 0, 0, 0],
    [9, -1, 1, -1, 0, 0, 0],
    [10, -1, 2, -1, 0, 0, 0],
    [11, -1, 3, -1, 0, 0, 0],

    [12, -2, 3, -1, 0, 0, 0],
    [13, -2, 2, -1, 0, 0, 0],
    [14, -2, 1, -1, 0, 0, 0],
    [15, -2, 0, -1, 0, 0, 0],
]


def gen_ring_of_markers(num_markers, radius, z):
    marker = 0
    angle = 0
    inc = 2 * math.pi / num_markers
    while marker < num_markers:
        yield [marker, radius * math.cos(angle), radius * math.sin(angle), z, angle, -math.pi/2, 0]
        marker += 1
        angle += inc


# Pool test #2: 4m diameter x 1m deep, markers on walls
pt2 = list(gen_ring_of_markers(num_markers=16, radius=2, z=-0.5))

worlds = [
    ['large.world', 'large_map.yaml', large_pool],
    ['small.world', 'small_map.yaml', small_pool],
    ['pt2.world', 'pt2_map.yaml', pt2],
]

for world in worlds:
    build_world(world[0], world[2])
    build_map(world[1], world[2])
