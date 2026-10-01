import argparse
import math

def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('finite number required')
    return value

def positive(value):
    value = finite(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('positive number required')
    return value

def parser():
    p = argparse.ArgumentParser(description='PX4 lab operator; coordinates ENU, yaw degrees CCW from East')
    p.add_argument('--timeout', type=positive, default=75.)
    p.add_argument('--json', action='store_true')
    commands = p.add_subparsers(dest='command', required=True)
    for name in ('status', 'arm', 'disarm', 'hold', 'land'):
        commands.add_parser(name)
    takeoff = commands.add_parser('takeoff')
    takeoff.add_argument('--height', type=positive, default=2.)
    goto = commands.add_parser('goto')
    for name in ('x', 'y', 'z'):
        goto.add_argument('--'+name, type=finite, required=True)
    goto.add_argument('--yaw', type=finite, default=0.)
    demo = commands.add_parser('demo')
    demo.add_argument('--runs', type=int, default=3)
    demo.add_argument('--hover', type=positive, default=10.)
    return p
