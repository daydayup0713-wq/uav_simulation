"""Rest-to-rest straight segment with bounded scalar speed and acceleration."""
import math
import numpy as np


class MotionProfile:
    def __init__(self,start,goal,speed,acceleration):
        self.start,self.goal=np.asarray(start,dtype=float),np.asarray(goal,dtype=float)
        if (self.start.shape!=(3,) or self.goal.shape!=(3,) or not np.isfinite([self.start,self.goal]).all()
                or not all(math.isfinite(v) and v>0 for v in (speed,acceleration))):
            raise ValueError('finite poses and positive speed/acceleration required')
        self.distance=float(np.linalg.norm(self.goal-self.start))
        self.direction=(self.goal-self.start)/self.distance if self.distance else np.zeros(3)
        self.acceleration=float(acceleration)
        self.peak=min(speed,math.sqrt(self.distance*acceleration))
        self.ramp=self.peak/acceleration
        self.cruise=max(0.,(self.distance-self.peak*self.ramp)/self.peak) if self.peak else 0.
        self.duration=2*self.ramp+self.cruise

    def sample(self,t):
        if not math.isfinite(t): raise ValueError('finite sample time required')
        t=max(0.,t)
        if t>=self.duration: return self.goal.copy(),np.zeros(3)
        if t<self.ramp:
            distance=.5*self.acceleration*t*t;speed=self.acceleration*t
        elif t<self.ramp+self.cruise:
            distance=.5*self.peak*self.ramp+self.peak*(t-self.ramp);speed=self.peak
        else:
            remaining=self.duration-t
            distance=self.distance-.5*self.acceleration*remaining*remaining;speed=self.acceleration*remaining
        return self.start+self.direction*distance,self.direction*speed
