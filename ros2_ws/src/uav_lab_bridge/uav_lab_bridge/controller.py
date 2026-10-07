"""ROS-independent flight policy with bounded motion and observed confirmation."""
from dataclasses import dataclass
import math
from .coordinate import step_toward

@dataclass
class Telemetry:
    position: tuple = (0., 0., 0.)
    yaw: float = 0.
    valid: bool = False
    armed: bool = False
    offboard: bool = False
    landing_mode: bool = False
    landed: bool = True
    preflight: bool = False
    position_at: float = -math.inf
    attitude_at: float = -math.inf
    status_at: float = -math.inf
    land_at: float = -math.inf

class FlightController:
    def __init__(self, speed=1., tolerance=.3, settle=2., warmup=2., acceleration=None):
        self.t = Telemetry()
        self.speed, self.tolerance, self.settle, self.warmup = speed, tolerance, settle, warmup
        self.state = 'WAITING'
        self.acceleration = acceleration
        self.profile, self.profile_time = None, 0.
        self.reason = 'waiting for telemetry'
        self.setpoint = None
        self.target = None
        self.yaw = 0.
        self.streaming = False
        self.results = {}
        self.active = None
        self.serial = 0
        self.commands = []
        self.pending = None
        self.since = 0.
        self.within = None
        self.takeoff_origin = None

    def update(self, now, **values):
        for key, value in values.items():
            setattr(self.t, key, value)
        if 'position' in values:
            self.t.position_at = now
        if 'yaw' in values:
            self.t.attitude_at = now
        if 'armed' in values or 'offboard' in values:
            self.t.status_at = now
        if 'landed' in values:
            self.t.land_at = now

    def fresh(self, now):
        return (self.t.valid and all(math.isfinite(v) for v in self.t.position)
                and now-self.t.position_at <= .5 and now-self.t.attitude_at <= .5
                # PX4 LandDetector publishes at 1Hz when unchanged. Allow
                # transport/scheduling jitter beyond that nominal interval.
                and now-self.t.status_at <= 1. and now-self.t.land_at <= 2.)

    def require_ready(self, now):
        if self.state == 'FAILSAFE' or not self.fresh(now):
            raise ValueError('telemetry unavailable or failsafe latched; restart lab')

    def begin(self, now):
        self.serial += 1
        self.active = self.serial
        self.since = now
        self.within = None
        return self.active

    def finish(self, success, reason):
        if self.active is not None:
            self.results[self.active] = (success, reason)
        self.active = None
        self.reason = reason
        # Result consumers are bounded to recent operations.
        if len(self.results) > 128:
            self.results.pop(min(self.results))

    def send(self, command, params, now, expectation):
        self.commands.append((command, params))
        self.pending = {'command': command, 'ack': False, 'deadline': now+5., 'expectation': expectation}

    def ack(self, command, result):
        if self.pending and command == self.pending['command']:
            if result == 0:
                self.pending['ack'] = True
            elif result != 5:  # IN_PROGRESS is not success.
                self.fail(f'PX4 rejected command {command}: result {result}')

    def drain_commands(self):
        commands, self.commands = self.commands, []
        return commands

    def fail(self, reason):
        self.finish(False, reason)
        self.pending = None
        self.commands.clear()
        self.streaming = False
        self.takeoff_origin = None
        self.state = 'FAILSAFE'

    def arm(self, now):
        self.require_ready(now)
        if self.active is not None or self.t.armed or not self.t.landed:
            raise ValueError('arm requires an idle, landed, disarmed vehicle')
        token = self.begin(now)
        self.setpoint = self.target = self.t.position
        self.takeoff_origin = self.t.position
        self.yaw = self.t.yaw
        self.streaming = True
        self.state = 'WARMUP'
        return token

    def disarm(self, now):
        self.require_ready(now)
        if not self.t.landed or self.active is not None:
            raise ValueError('disarm requires landed vehicle and no active operation')
        token = self.begin(now)
        self.state = 'DISARMING'
        self.takeoff_origin = None
        self.send(400, (0.,), now, 'disarmed')
        return token

    def hold(self, now):
        self.require_ready(now)
        if self.state == 'LANDING' or not self.t.armed or not self.t.offboard:
            raise ValueError('hold requires an armed Offboard vehicle outside landing')
        self.finish(False, 'interrupted by hold')
        self.pending = None
        self.setpoint = self.target = self.t.position
        self.yaw = self.t.yaw
        self.state = 'HOLDING'
        self.streaming = True

    def fly(self, operation, now, target=None, yaw=None, height=2.):
        self.require_ready(now)
        if operation not in ('TAKEOFF', 'GOTO', 'LAND'):
            raise ValueError('unknown operation')
        if not self.t.armed:
            raise ValueError('explicit arm required')
        if self.state == 'LANDING':
            raise ValueError('landing already in progress')
        if operation == 'LAND':
            self.takeoff_origin = None
            self.finish(False, 'interrupted by land')
            token = self.begin(now)
            self.state = 'LANDING'
            self.send(21, (), now, 'landing')
            return token
        if self.active is not None or not self.t.offboard:
            raise ValueError('motion requires idle Offboard vehicle')
        if operation == 'TAKEOFF':
            if self.takeoff_origin is None or math.dist(self.t.position, self.takeoff_origin) > .3:
                raise ValueError('takeoff requires explicit ground arm and remaining within 0.3m of its origin')
            target = (self.takeoff_origin[0], self.takeoff_origin[1], self.takeoff_origin[2]+height)
        if target is None or len(target) != 3 or not all(math.isfinite(v) for v in target):
            raise ValueError('finite target required')
        if abs(target[0]) > 10 or abs(target[1]) > 10 or not .2 <= target[2] <= 5:
            raise ValueError('target outside lab bounds: x/y +/-10m, z 0.2..5m')
        if yaw is not None and not math.isfinite(yaw):
            raise ValueError('finite yaw required')
        token = self.begin(now)
        self.takeoff_origin = None
        self.target = tuple(target)
        self.setpoint = self.t.position
        if self.acceleration is not None:
            from .trajectory import MotionProfile
            self.profile = MotionProfile(self.setpoint, self.target, self.speed, self.acceleration)
            self.profile_time = 0.
        self.yaw = self.t.yaw if yaw is None else yaw
        self.streaming = True
        self.state = 'MOVING'
        return token

    def cancel(self, token, now):
        if token != self.active or self.state != 'MOVING':
            return False
        self.hold(now)
        return True

    def tick(self, now, sim_dt):
        if self.state == 'FAILSAFE':
            return
        if not self.fresh(now):
            if self.streaming or self.active is not None:
                self.fail('telemetry stale or position invalid')
            else:
                self.state = 'WAITING'
            return
        if self.state == 'WAITING':
            self.state = 'READY'
            self.reason = 'telemetry ready'
        if self.state == 'WARMUP' and now-self.since >= self.warmup:
            self.state = 'ENTERING_OFFBOARD'
            self.send(176, (1., 6.), now, 'offboard')
        if self.pending:
            p = self.pending
            confirmed = {'offboard': self.t.offboard and self.t.preflight, 'armed': self.t.armed,
                         'disarmed': not self.t.armed,
                         'landing': self.t.landing_mode or self.t.landed and not self.t.armed}[p['expectation']]
            if p['ack'] and confirmed:
                self.pending = None
                if self.state == 'ENTERING_OFFBOARD':
                    self.state = 'ARMING'
                    self.send(400, (1.,), now, 'armed')
                elif self.state == 'ARMING':
                    self.state = 'HOLDING'
                    self.finish(True, 'armed and Offboard confirmed')
                elif self.state == 'DISARMING':
                    self.streaming = False
                    self.state = 'READY'
                    self.finish(True, 'disarmed confirmed')
            elif now > p['deadline']:
                self.fail('command ACK/state confirmation timeout')
                return
        if self.state == 'LANDING':
            if self.pending is None and self.t.landed and not self.t.armed:
                self.streaming = False
                self.state = 'READY'
                self.finish(True, 'landed and disarmed confirmed')
            elif now-self.since > 60:
                self.fail('landing timeout')
            return
        if self.state in ('MOVING', 'HOLDING') and (not self.t.offboard or not self.t.armed):
            self.fail('vehicle left armed Offboard state')
            return
        if self.state == 'MOVING':
            if self.acceleration is None:
                self.setpoint = step_toward(self.setpoint, self.target, self.speed*min(max(sim_dt, 0), .1))
            else:
                self.profile_time += min(max(sim_dt, 0), .1)
                self.setpoint = tuple(self.profile.sample(self.profile_time)[0])
            yaw_error = abs(math.atan2(math.sin(self.t.yaw-self.yaw), math.cos(self.t.yaw-self.yaw)))
            if math.dist(self.t.position, self.target) <= self.tolerance and yaw_error <= .15:
                if self.within is None:
                    self.within = now
                if now-self.within >= self.settle and math.dist(self.setpoint, self.target) < 1e-9:
                    self.state = 'HOLDING'
                    self.finish(True, 'target reached within tolerance continuously')
            else:
                self.within = None
            if self.active is not None and now-self.since > 60:
                self.fail('motion timeout')
