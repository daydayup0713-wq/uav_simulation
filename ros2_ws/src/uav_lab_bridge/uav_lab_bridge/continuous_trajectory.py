"""C2 minimum-jerk quintics with analytic limits and conservative curve checks.

Coefficients are ascending powers of normalized segment time u in [0,1].
Waypoint derivatives minimize the integral of squared jerk; explicit dwell knots
constrain both velocity and acceleration to zero. No tracking/physics is faked.
"""
from dataclasses import dataclass, field, asdict
import math
import numpy as np
from numpy.polynomial import polynomial as poly


def vector(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('finite three-dimensional vector required')
    return value


@dataclass(frozen=True)
class Limits:
    speed: float = .5
    acceleration: float = .5
    jerk: float = 1.

    def validate(self):
        if not all(math.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('positive finite trajectory limits required')


@dataclass
class State:
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    jerk: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.
    yaw_rate: float = 0.


@dataclass
class Segment:
    duration: float
    coefficients: np.ndarray
    yaw_coefficients: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def __post_init__(self):
        self.coefficients = np.asarray(self.coefficients, dtype=float)
        self.yaw_coefficients = np.asarray(self.yaw_coefficients, dtype=float)
        if (not math.isfinite(self.duration) or not 0 < self.duration <= 3600
                or self.coefficients.shape != (3, 6) or self.yaw_coefficients.shape != (6,)
                or not np.isfinite(self.coefficients).all() or not np.isfinite(self.yaw_coefficients).all()):
            raise ValueError('finite quintic coefficients and bounded positive duration required')

    def sample(self, t):
        if not math.isfinite(t):
            raise ValueError('finite sample time required')
        u = np.clip(t / self.duration, 0., 1.)
        values = [np.array([poly.polyval(u, poly.polyder(c, m)) for c in self.coefficients])
                  / self.duration ** m for m in range(4)]
        return State(*values, yaw=float(poly.polyval(u, self.yaw_coefficients)),
                     yaw_rate=float(poly.polyval(u, poly.polyder(self.yaw_coefficients))) / self.duration)

    def maximum_derivative(self, order):
        if order not in (1, 2, 3):
            raise ValueError('derivative order must be 1, 2 or 3')
        derivatives = [poly.polyder(c, order) / self.duration ** order for c in self.coefficients]
        squared = np.zeros(1)
        for derivative in derivatives:
            squared = poly.polyadd(squared, poly.polymul(derivative, derivative))
        roots = poly.polyroots(poly.polyder(squared))
        times = [0., 1.] + [float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 <= r.real <= 1]
        return max(float(np.linalg.norm([poly.polyval(t, d) for d in derivatives])) for t in times)

    def bezier(self):
        # Power-to-Bernstein conversion, whose convex hull contains the curve.
        return np.array([sum((self.coefficients[:, k] * math.comb(i, k) / math.comb(5, k)
                              for k in range(i + 1)), np.zeros(3)) for i in range(6)])


def quintic(p0, p1, v0, v1, a0, a1, duration):
    first = np.stack([p0, v0 * duration, a0 * duration ** 2 / 2], axis=1)
    rest = np.stack([p1 - first.sum(axis=1), v1 * duration - first[:, 1] - 2 * first[:, 2],
                     a1 * duration ** 2 - 2 * first[:, 2]])
    last = np.linalg.solve(np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rest).T
    return np.concatenate([first, last], axis=1)


def minimum_jerk(points, durations, initial_v, initial_a, stop_knots):
    count, dimensions = points.shape
    velocities, accelerations = np.zeros_like(points), np.zeros_like(points)
    velocities[0], accelerations[0] = initial_v, initial_a
    variables = [(k, d) for k in range(1, count - 1) if k not in stop_knots for d in (1, 2)]
    nodes, weights = np.polynomial.legendre.leggauss(3)
    u, weights = (nodes + 1) / 2, weights / 2

    def residual(v, a):
        rows = []
        for i, duration in enumerate(durations):
            coeff = quintic(points[i], points[i + 1], v[i], v[i + 1], a[i], a[i + 1], duration)
            rows.extend([[poly.polyval(t, poly.polyder(c, 3)) * math.sqrt(w / duration ** 5)
                          for c in coeff] for t, w in zip(u, weights)])
        return np.array(rows)

    base = residual(velocities, accelerations)
    if variables:
        # Objective is linear in knot derivatives. Three-point quadrature is exact
        # for the squared quadratic jerk of a quintic, rather than an approximation.
        columns = []
        for knot, derivative in variables:
            v, a = velocities.copy(), accelerations.copy()
            (v if derivative == 1 else a)[knot, :] += 1
            columns.append((residual(v, a) - base)[:, 0])
        design = np.array(columns).T
        result = np.linalg.lstsq(design, -base, rcond=1e-12)[0]
        for (knot, derivative), row in zip(variables, result):
            (velocities if derivative == 1 else accelerations)[knot] = row
    return [Segment(float(t), quintic(points[i], points[i + 1], velocities[i], velocities[i + 1],
                                      accelerations[i], accelerations[i + 1], t))
            for i, t in enumerate(durations)]


class Trajectory:
    def __init__(self, segments, limits=Limits()):
        self.segments, self.limits = tuple(segments), limits
        limits.validate()
        if not self.segments or len(self.segments) > 512:
            raise ValueError('one to 512 trajectory segments required')
        self.ends = np.cumsum([s.duration for s in self.segments])
        self.duration = float(self.ends[-1])
        if self.duration > 3600:
            raise ValueError('trajectory exceeds one-hour duration budget')
        for left, right in zip(self.segments, self.segments[1:]):
            a, b = left.sample(left.duration), right.sample(0.)
            if any(not np.allclose(getattr(a, k), getattr(b, k), rtol=0, atol=1e-6)
                   for k in ('position', 'velocity', 'acceleration')):
                raise ValueError('trajectory must be C2 continuous')
            if abs(a.yaw - b.yaw) > 1e-6 or abs(a.yaw_rate - b.yaw_rate) > 1e-6:
                raise ValueError('yaw must be continuous')

    @classmethod
    def generate(cls, points, limits=Limits(), initial=None, dwell=None, yaws=None):
        limits.validate()
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or not 2 <= len(points) <= 128
                or not np.isfinite(points).all()):
            raise ValueError('two to 128 finite route controls required')
        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(distances < 1e-5):
            raise ValueError('successive route controls must be distinct')
        initial = initial or State(points[0])
        v, a = vector(initial.velocity), vector(initial.acceleration)
        if (not np.allclose(vector(initial.position), points[0], rtol=0, atol=1e-7)
                or np.linalg.norm(v) > limits.speed or np.linalg.norm(a) > limits.acceleration
                or not math.isfinite(initial.yaw) or not math.isfinite(initial.yaw_rate)):
            raise ValueError('initial trajectory state is incompatible with route or limits')
        dwell = dwell or {}
        if any(not isinstance(k, int) or not 0 < k < len(points) - 1 or not math.isfinite(t)
               or not 0 < t <= 600 for k, t in dwell.items()):
            raise ValueError('positive bounded dwell at an intermediate control required')
        durations = np.maximum(distances / limits.speed, .5)
        for _ in range(40):
            segments = minimum_jerk(points, durations, v, a, set(dwell))
            ratio = max(max(s.maximum_derivative(1) / limits.speed,
                            math.sqrt(s.maximum_derivative(2) / limits.acceleration),
                            (s.maximum_derivative(3) / limits.jerk) ** (1 / 3)) for s in segments)
            if ratio <= 1 + 1e-9:
                break
            durations *= max(1.02, ratio * 1.01)
        else:
            raise ValueError('trajectory cannot meet derivative limits from initial state')
        angles = np.full(len(points), initial.yaw) if yaws is None else np.unwrap(np.asarray(yaws, dtype=float))
        if angles.shape != (len(points),) or not np.isfinite(angles).all() or abs(angles[0] - initial.yaw) > 1e-6:
            raise ValueError('finite continuous initial yaw and per-control headings required')
        yaw_points = np.column_stack([angles, np.zeros((len(points), 2))])
        yaw_segments = minimum_jerk(yaw_points, durations, np.array([initial.yaw_rate, 0., 0.]),
                                    np.zeros(3), set(dwell))
        result = []
        for i, (segment, yaw_segment) in enumerate(zip(segments, yaw_segments)):
            segment.yaw_coefficients = yaw_segment.coefficients[0]
            result.append(segment)
            if i + 1 in dwell:
                coeff = np.zeros((3, 6)); coeff[:, 0] = points[i + 1]
                heading = np.zeros(6); heading[0] = angles[i + 1]
                result.append(Segment(dwell[i + 1], coeff, heading))
        return cls(result, limits)

    def sample(self, t):
        if not math.isfinite(t):
            raise ValueError('finite sample time required')
        index = min(int(np.searchsorted(self.ends, max(0., t), side='right')), len(self.segments) - 1)
        offset = 0. if index == 0 else self.ends[index - 1]
        return self.segments[index].sample(max(0., t) - offset)

    @classmethod
    def stop(cls, initial, limits=Limits()):
        limits.validate()
        p, v, a = vector(initial.position), vector(initial.velocity), vector(initial.acceleration)
        if (np.linalg.norm(v) > limits.speed or np.linalg.norm(a) > limits.acceleration
                or not math.isfinite(initial.yaw) or not math.isfinite(initial.yaw_rate)):
            raise ValueError('initial stop state exceeds limits')
        base = max(1., 2 * np.linalg.norm(v) / limits.acceleration,
                   math.sqrt(6 * np.linalg.norm(v) / limits.jerk), 6 * np.linalg.norm(a) / limits.jerk)
        for factor in (1., 1.1, 1.25, 1.5, 2., 3., 4., 6.):
            duration = float(base * factor)
            goal = p + v * duration / 2 + a * duration ** 2 / 12
            coefficients = quintic(p, goal, v, np.zeros(3), a, np.zeros(3), duration)
            yaw_coeff = quintic(np.array([initial.yaw]), np.array([initial.yaw + initial.yaw_rate * duration / 2]),
                                np.array([initial.yaw_rate]), np.zeros(1), np.zeros(1), np.zeros(1), duration)[0]
            result = cls([Segment(duration, coefficients, yaw_coeff)], limits)
            maxima = result.derivative_maxima()
            if all(maxima[k] <= value + 1e-8 for k, value in asdict(limits).items()):
                return result
        raise ValueError('cannot construct a constrained stop from initial state')

    def derivative_maxima(self):
        return {name: max(s.maximum_derivative(order) for s in self.segments)
                for order, name in enumerate(('speed', 'acceleration', 'jerk'), 1)}

    def collision_free(self, collision, max_nodes=50000, start_time=0., end_time=None):
        if not isinstance(max_nodes, int) or max_nodes <= 0:
            raise ValueError('positive curve subdivision budget required')
        end_time = self.duration if end_time is None else end_time
        if not math.isfinite(start_time) or not math.isfinite(end_time) or not 0 <= start_time <= end_time <= self.duration + 1e-9:
            raise ValueError('ordered collision interval within trajectory required')

        def split(hull, u):
            levels = [hull]
            while len(levels[-1]) > 1:
                levels.append(levels[-1][:-1] * (1 - u) + levels[-1][1:] * u)
            return np.array([level[0] for level in levels]), np.array([level[-1] for level in reversed(levels)])

        queue, offset = [], 0.
        for segment in self.segments:
            lo, hi = max(0., (start_time - offset) / segment.duration), min(1., (end_time - offset) / segment.duration)
            if lo <= hi:
                hull = segment.bezier()
                if lo > 0:
                    _, hull = split(hull, min(1., lo))
                if hi < 1:
                    hull, _ = split(hull, (hi - lo) / (1 - lo) if lo < 1 else 0.)
                queue.append((hull, 0))
            offset += segment.duration
        visited = 0
        while queue:
            hull, depth = queue.pop(); visited += 1
            if visited > max_nodes:
                return False
            lower, upper = hull.min(axis=0), hull.max(axis=0)
            lo = np.floor((lower - collision.lower) / collision.resolution - 1e-9).astype(int)
            hi = np.floor((upper - collision.lower) / collision.resolution + 1e-9).astype(int)
            if np.any(lo < 0) or np.any(hi >= collision.free.shape):
                if depth >= 20:
                    return False
            elif collision.free[tuple(slice(l, h + 1) for l, h in zip(lo, hi))].all():
                continue
            if depth >= 20 or np.linalg.norm(upper - lower) < collision.resolution * 1e-5:
                return False
            left, right = split(hull, .5)
            queue.append((left, depth + 1))
            queue.append((right, depth + 1))
        return True

    def to_dict(self):
        return {'schema': 1, 'frame': 'odom', 'limits': asdict(self.limits),
                'segments': [{'duration': s.duration, 'coefficients': s.coefficients.tolist(),
                              'yaw_coefficients': s.yaw_coefficients.tolist()} for s in self.segments]}

    @classmethod
    def from_dict(cls, payload):
        if payload.get('schema') != 1 or payload.get('frame') != 'odom':
            raise ValueError('unsupported trajectory schema/frame')
        result = cls([Segment(s['duration'], s['coefficients'], s['yaw_coefficients'])
                      for s in payload['segments']], Limits(**payload['limits']))
        maxima = result.derivative_maxima()
        if any(maxima[k] > v + 1e-7 for k, v in asdict(result.limits).items()):
            raise ValueError('trajectory derivative limit exceeded')
        end = result.sample(result.duration)
        if np.linalg.norm(end.velocity) > 1e-6 or np.linalg.norm(end.acceleration) > 1e-6:
            raise ValueError('trajectory must end at rest')
        return result
