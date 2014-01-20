#!/usr/bin/env python
from __future__ import print_function

import math
import sys

scheme="""<?xml version='1.0'?>
<sdf version='1.4'>
  <model name='Levine'>
    <link name='Floor_0'>
      <collision name='Floor_0_Collision'>
        <geometry>
          <box>
            <size>40 30 0.1</size>
          </box>
        </geometry>
        <pose>0 0 0.05 0 -0 0</pose>
      </collision>
      <visual name='Floor_0_Visual'>
        <pose>0 0 0.05 0 -0 0</pose>
        <geometry>
          <box>
            <size>40 30 0.1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0</linear>
        <angular>0</angular>
      </velocity_decay>
      <pose>15 10 1.5 0 -0 0</pose>
    </link>
{links}
    <static>1</static>
  </model>
</sdf>"""

wall_scheme="""    <link name='Wall_{idx}'>
      <collision name='Wall_{idx}_Collision'>
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
        <pose>0 0 {half_height} 0 0 0</pose>
      </collision>
      <visual name='Wall_{idx}_Visual'>
        <pose>0 0 {half_height} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0</linear>
        <angular>0</angular>
      </velocity_decay>
      <pose>{x} {y} 0 0 0 {theta}</pose>
    </link>"""

def wall(idx, x1, y1, x2, y2):
    x = (x1 + x2) / 2.0
    y = (y1 + y2) / 2.0
    thickness = 0.2
    height = 2.5
    length = math.hypot(x2 - x1, y2 - y1)
    theta = math.atan2(y2 - y1, x2 - x1)
    half_height = height / 2.0
    id = 1
    return wall_scheme.format(**locals())

def add_wall(links, points, x_offset = 0.0):
    for idx, (p1, p2) in enumerate(zip(points, points[1:]), len(links)):
        links.append(wall(idx, p1[0] + x_offset, p1[1], p2[0] + x_offset, p2[1]))
    return links

hallway_width = 1.6

out_left = -0.6
out_right = 9.7
ytop = 24.6
out_bot = -1.0

xl = -0.6
door_height = 0.9
out_left = ((xl,  0.6),
            (xl,  5.5),
            (xl-0.1,  5.5),
            (xl-0.1,  5.5+door_height),
            (xl,  5.5+door_height),
            (xl,  6.9),
            (xl-0.1,  6.9),
            (xl-0.1,  6.9+door_height),
            (xl,  6.9+door_height),
            (xl, 15.2),
            (xl-0.1, 15.2),
            (xl-0.1, 15.2+door_height),
            (xl, 15.2+door_height),
            (xl, 16.8),
            (xl-0.1, 16.8),
            (xl-0.1, 16.8+door_height),
            (xl, 16.8+door_height),
            (xl, ytop))

xr = 9.3
out_top = ((xl, ytop),
           (12.6, ytop),
           (12.6, ytop - hallway_width),
           (xr, ytop-hallway_width))

out_right = ((xr, 0.6),
             (xr, 5.5),
             (xr+0.1, 5.5),
             (xr+0.1, 5.5+door_height),
             (xr, 5.5+door_height),
             (xr, 6.8),
             (xr+0.1, 6.8),
             (xr+0.1, 6.8+door_height),
             (xr, 6.8+door_height),
             (xr, 15.3),
             (xr+0.1, 15.3),
             (xr+0.1, 15.3+door_height),
             (xr, 15.3+door_height),
             (xr, 16.6),
             (xr+0.1, 16.6),
             (xr+0.1, 16.6+door_height),
             (xr, 16.6+door_height),
             (xr, ytop-hallway_width))

ybot = 0.6 - hallway_width
out_bottom = ((xl, 0.6),
              (-3.5, 0.6),
              (-3.5, 0.5),

              (-3.5-0.1, 0.5), # door
              (-3.5-0.1, 0.5-door_height),
              (-3.5, 0.5-door_height),
              (-3.5, ybot),

              (-2.8, ybot), # door
              (-2.8, ybot-0.1),
              (-2.8+door_height, ybot-0.1),
              (-2.8+door_height, ybot),
              (15.8, ybot),

              (15.8, -1.4), # Levine by the elevators
              (25.8, -1.4),
              (25.8, -1.6),
              (29.6, -1.6),
              (29.6, 0.0),
              (28.6, 0.0),
              (28.6, 0.6),
              out_right[0])

in_leftx = xl+hallway_width
in_rightx = xr-hallway_width
in_topy = 9.1
in_boty = 0.6

in_bot = ((in_leftx, in_boty),
          (in_leftx, 4), # door
          (in_leftx+0.1, 4),
          (in_leftx+0.1, 4 + door_height),
          (in_leftx, 4 + door_height),
          (in_leftx, 5.4),
          (in_leftx+0.1, 5.4),
          (in_leftx+0.1, 5.4+door_height),
          (in_leftx, 5.4+door_height),
          (in_leftx, in_topy),
          (in_leftx+0.5, in_topy),
          (in_leftx+0.5, in_topy+0.3),
          (in_leftx+1.5, in_topy+0.3),
          (in_leftx+1.5, in_topy),
          (in_rightx, in_topy),
          (in_rightx, 6.3),
          (in_rightx-0.1, 6.3),
          (in_rightx-0.1, 6.3-door_height),
          (in_rightx, 6.3-door_height),
          (in_rightx, 4.9),
          (in_rightx-0.1, 4.9),
          (in_rightx-0.1, 4.9-door_height),
          (in_rightx, 4.9-door_height),
          (in_rightx, in_boty),
          (4.5, in_boty),
          (4.5, in_boty+0.1),
          (4.5-door_height, in_boty+0.1),
          (4.5-door_height, in_boty),
          (in_leftx, in_boty))

in_topy = ytop-hallway_width
in_boty = 13.8

in_top = ((in_leftx, in_boty),
          (in_leftx, 16.8), # door
          (in_leftx+0.1, 16.8),
          (in_leftx+0.1, 16.8 + door_height),
          (in_leftx, 16.8 + door_height),
          (in_leftx, 18.1),
          (in_leftx+0.1, 18.1),
          (in_leftx+0.1, 18.1+door_height),
          (in_leftx, 18.1+door_height),
          (in_leftx, in_topy),
          (in_rightx, in_topy),
          (in_rightx, 19.0),
          (in_rightx-0.1, 19.0),
          (in_rightx-0.1, 19.0-door_height),
          (in_rightx, 19.0-door_height),
          (in_rightx, 17.5),
          (in_rightx-0.1, 17.5),
          (in_rightx-0.1, 17.5-door_height),
          (in_rightx, 17.5-door_height),
          (in_rightx, in_boty-0.4),
          (in_rightx-1.0, in_boty-0.4),
          (in_rightx-1.0, in_boty),
          (in_leftx, in_boty))

if __name__ == "__main__":
    links = []
    add_wall(links, out_left)
    add_wall(links, out_top)
    add_wall(links, out_right)
    add_wall(links, out_bottom)
    add_wall(links, in_bot)
    add_wall(links, in_top)
    print(scheme.format(links='\n'.join(links)))
