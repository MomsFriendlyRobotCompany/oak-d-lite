
from std_msgs.msg import Header
from collections import namedtuple

# Vector3 = namedtuple("Vector3","x y z")
# Quaternion = namedtuple("Quaternion","w x y z")

# class Header:
#     stamp = 0
#     frame_id = None

#     def __init__(self):
#         pass

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Quaternion:
    def __init__(self, w=0, x=0, y=0, z=0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

class Imu:
    def __init__(self):
        self.header = Header()
        self.linear_acceleration = Vector3()
        self.angular_velocity = Vector3()
        self.orientation = Quaternion()

class ChannelFloat32:
    def __init__(self):
        self.name = None
        self.values = []

class PointCloud:
    def __init__(self):
        self.header = Header()
        self.points = []   # Vector3
        self.channels = [] # ChannelFloat32

    def __str__(self):
        id = self.header.frame_id
        s = f"{id} points: {len(self.points)}"
        for i, c in enumerate(self.channels):
            s += f"\n  channel[{i}] {c.name}: {len(c.values)}"
        return s

# class PointCloud2:
#     def __init__(self):
#         header = Header()
#         height
#         width
#         fields
#         is_bigendian = False
#         point_step
#         row_step
#         data
#         is_dense = True