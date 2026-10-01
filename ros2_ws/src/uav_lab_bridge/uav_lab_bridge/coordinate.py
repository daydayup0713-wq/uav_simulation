"""Frame conversions. Quaternions are xyzw in ROS and wxyz in PX4."""
import math

def enu_to_ned(v):
    return (float(v[1]), float(v[0]), -float(v[2]))

ned_to_enu = enu_to_ned

def yaw_to_ned(yaw):
    return math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))

def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)

def normalized(q):
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    sign = -1 if q[0] < 0 else 1
    return tuple(sign*v/norm for v in q)

def px4_to_ros_quaternion(q):
    s = math.sqrt(.5)
    w, x, y, z = normalized(multiply(multiply((0, s, s, 0), normalized(q)), (0, 1, 0, 0)))
    return (x, y, z, w)

def ros_to_px4_quaternion(q):
    x, y, z, w = q
    s = math.sqrt(.5)
    return normalized(multiply(multiply((0, s, s, 0), normalized((w, x, y, z))), (0, 1, 0, 0)))

def step_toward(start, target, max_distance):
    distance = math.dist(start, target)
    if distance <= max_distance or distance == 0:
        return tuple(target)
    return tuple(a+(b-a)*max_distance/distance for a, b in zip(start, target))
