"""Map unshifted PX4 microseconds to the Gazebo clock, without epoch guesses."""
class Px4Clock:
    def __init__(self):
        self.px4_us = None
        self.ros_ns = None

    def observe(self, px4_us, ros_ns):
        if self.px4_us is not None and (px4_us < self.px4_us or ros_ns < self.ros_ns):
            raise RuntimeError('clock regression: restart the lab')
        self.px4_us, self.ros_ns = int(px4_us), int(ros_ns)

    def timestamp(self, ros_ns):
        if self.px4_us is None:
            raise RuntimeError('PX4 clock not observed')
        if ros_ns < self.ros_ns:
            raise RuntimeError('clock regression: restart the lab')
        return self.px4_us + (int(ros_ns)-self.ros_ns)//1000
