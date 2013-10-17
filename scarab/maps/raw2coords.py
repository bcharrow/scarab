#!/usr/bin/python
import sys
import yaml

with open(sys.argv[1]) as f:
    lines = f.readlines()
data = ''.join(lines)

data = yaml.load(data)

x_orig, y_orig, z_orig = data['origin']
x, y = sys.argv[2:4]
x, y = float(x), float(y)
print "Coordinates: x=% 0.2f y=% 0.2f" % (
    x * data['resolution'] + x_orig,
    y * data['resolution'] + y_orig)

