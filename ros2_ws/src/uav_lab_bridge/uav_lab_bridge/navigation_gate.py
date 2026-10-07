"""Fail-closed source-time navigation heartbeat; a live link never auto-recovers."""
import math


class NavigationGate:
    def __init__(self):
        self.received=-math.inf;self.source=None;self.healthy=False;self.was_ready=False;self.failed=''

    def observe(self,ready,source,frame,now):
        self.received=now;self.healthy=ready and frame=='odom' and math.isfinite(source)
        self.source=source

    def ready(self,now,sim):
        valid=(not self.failed and self.healthy and now-self.received<=.75 and self.source is not None and -.1<=sim-self.source<=1.)
        if self.was_ready and not valid:self.failed='navigation map/heartbeat expired or invalid; restart lab'
        if valid:self.was_ready=True
        return bool(valid)
